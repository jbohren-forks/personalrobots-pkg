/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Stuart Glaser
 */
#include "urdf/parser.h"
#include <map>
#include <vector>
#include <cmath>
#include <sstream>
#include "tinyxml/tinyxml.h"
#include "string_utils/string_utils.h"
#include "math_expr/MathExpression.h"


namespace urdf {


bool queryVectorAttribute(TiXmlElement *el, const char *name, std::vector<double> *value)
{
  value->clear();
  const char *s = el->Attribute(name);
  if (!s)
    return false;

  std::vector<std::string> pieces;
  string_utils::split(s, pieces);
  for (unsigned int i = 0; i < pieces.size(); ++i)
    value->push_back(atof(pieces[i].c_str()));

  return true;
}

template <class C>
void deleteValues(C *c)
{
  typename C::iterator it;
  for (it = c->begin(); it != c->end(); ++it)
  {
    if (it->second != NULL)
    {
      delete it->second;
      it->second = NULL;
    }
  }
}

// Pulls the const and const_block elements out of the XML file.
class ConstsAndBlocks : public TiXmlVisitor
{
public:
  ConstsAndBlocks() {}
  virtual ~ConstsAndBlocks()
  {
    deleteValues(&blocks);
  }

  virtual bool VisitEnter(const TiXmlElement &elt, const TiXmlAttribute *)
  {
    if (elt.Value() == std::string("const"))
    {
      if (!elt.Attribute("name"))
      {
        fprintf(stderr, "Invalid const declaration: no name\n");
        return false;
      }
      if (!elt.Attribute("value"))
      {
        fprintf(stderr, "Invalid const declaration: %s has no value\n", elt.Attribute("name"));
        return false;
      }

      consts.insert(std::make_pair(elt.Attribute("name"), elt.Attribute("value")));
      return false;
    }
    else if (elt.Value() == std::string("const_block"))
    {
      if (!elt.Attribute("name"))
      {
        fprintf(stderr, "Invalid const_block declaration: no name\n");
        return false;
      }

      blocks.insert(std::make_pair(elt.Attribute("name"), elt.Clone()->ToElement()));
      return false;
    }

    return true;
  }

  // NULL if not found
  TiXmlElement *getBlock(const std::string &name)
  {
    std::map<std::string,TiXmlElement*>::iterator it;
    it = blocks.find(name);
    return it == blocks.end() ? NULL : it->second;
  }

  // Empty string if not found
  std::string getConst(const std::string &name)
  {
    std::map<std::string,std::string>::iterator it;
    it = consts.find(name);
    return it == consts.end() ? "" : it->second;
  }

private:
  std::map<std::string,std::string> consts;
  std::map<std::string,TiXmlElement*> blocks;
};

double evalConstant(void *v, std::string &s)
{
  ConstsAndBlocks *c = (ConstsAndBlocks*)v;

  std::string const_str = c->getConst(s);
  if (const_str == "")
    return NAN;

  return meval::EvaluateMathExpression(const_str, evalConstant, v);
}

// Preorder traversal
TiXmlElement *next_element(TiXmlElement *e)
{
  if (!e)
    return NULL;
  if (e->FirstChildElement())
    return e->FirstChildElement();

  while (e && !e->NextSiblingElement())
  {
    e = e->Parent() ? e->Parent()->ToElement() : NULL;
  }
  return e ? e->NextSiblingElement() : NULL;
}

std::string normalizeText(const std::string &text, ConstsAndBlocks &lookup)
{
  using namespace std;
  stringstream ss;
  vector<string> pieces;
  string_utils::split(text, pieces);

  // Evaluates each piece.
  for (unsigned int i = 0; i < pieces.size(); ++i)
  {
    double val = meval::EvaluateMathExpression(pieces[i], evalConstant, &lookup);

    if (i > 0)
      ss << " ";

    if (isnan(val))
      ss << pieces[i];
    else
      ss << val;
  }

  return ss.str();
}

// Replaces the element elt with the node replacements and its
// siblings.
void replaceReference(TiXmlElement **to_replace, TiXmlNode *replacements)
{
  TiXmlNode *inserted = *to_replace;
  while (replacements)
  {
    inserted = inserted->Parent()->InsertAfterChild(inserted, *replacements);
    assert(inserted);
    replacements = replacements->NextSibling();
  }

  (*to_replace)->Parent()->RemoveChild(*to_replace);
  *to_replace = NULL;
}

bool normalizeXml(TiXmlElement *xml)
{
  TiXmlElement* elt = xml;

  using namespace std;

  TiXmlElement *previous = NULL;  // tracks the element before elt

  //------------------------------------------------------------
  // Zeroeth Pass

  // - Replaces include elements with the desired file
  while (elt)
  {
    if (elt->ValueStr() == std::string("include"))
    {
      std::string filename(elt->GetText());

      // FIXME: get path of current xml node and use relative path for includes
      std::string currentPath = getenv("MC_RESOURCE_PATH") ? getenv("MC_RESOURCE_PATH") : ".";
      if (currentPath!="") filename = (currentPath+"/")+filename;

      TiXmlDocument doc(filename);
      doc.LoadFile();

      if (!doc.RootElement())
      {
        fprintf(stderr, "Included file not found: %s\n", filename.c_str());
        elt->Parent()->RemoveChild(elt);
      }
      else
      {
        replaceReference(&elt, doc.RootElement()->FirstChild());
      }

      elt = next_element(previous);
    }
    else
    {
      previous = elt;
      elt = next_element(elt);
    }
  }

  //------------------------------------------------------------
  // First Pass

  ConstsAndBlocks lookup;
  xml->Accept(&lookup);

  //------------------------------------------------------------
  // Second Pass (destructive)

  previous = NULL;
  elt = xml;
  // - Removes const and const_block elements.
  // - Replaces insert_const_block elements with the corresponding xml tree.
  // - Replaces const strings with the actual values.
  // - Reduces mathematical expressions.
  while (elt)
  {
    // Kills elt, if it's a const or const_block definition
    if (elt->ValueStr() == std::string("const") || elt->ValueStr() == std::string("const_block"))
    {
      elt->Parent()->RemoveChild(elt);
      elt = next_element(previous);
    }
    // Replaces elt if it's an insert_const_block tag.
    else if (elt->ValueStr() == std::string("insert_const_block"))
    {
      const char *name = elt->Attribute("name");
      TiXmlNode *replacements = name ? lookup.getBlock(name)->FirstChild() : NULL;

      if (replacements)
      {
        replaceReference(&elt, replacements);
      }
      else
      {
        elt->Parent()->RemoveChild(elt);
        fprintf(stderr, "No const_block named \"%s\"\n", name);
      }

      elt = next_element(previous);
    }
    // Replaces the attributes of elt and the text inside elt if they
    // have constants or expressions, and continues down the tree.
    else
    {
      for (TiXmlAttribute *att = elt->FirstAttribute(); att; att = att->Next())
      {
        att->SetValue(normalizeText(att->ValueStr(), lookup));
      }

      TiXmlText *text = elt->FirstChild() ? elt->FirstChild()->ToText() : NULL;
      if (text)
        text->SetValue(normalizeText(text->ValueStr(), lookup));

      previous = elt;
      elt = next_element(elt);
    }
  }

  return true;
}

} // namespace urdf
