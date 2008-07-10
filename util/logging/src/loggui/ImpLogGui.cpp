#include "LogGui.h"

#include <wx/app.h>
#include <wx/timer.h>
#include <wx/dirdlg.h>

#include "ros/node.h"
#include "std_msgs/String.h"

#include <stdio.h>

typedef std::vector<std::pair<std::string, std::string> > topicList;

class treeData : public wxTreeItemData
{
public:
  std::string name;
  bool save;
};

class ImpLogGui : public LogGui, public ros::node
{
  wxTimer* timer;
  std_msgs::String msg;

public:
  ImpLogGui(wxWindow* parent) : LogGui(parent), ros::node("LogGui")
  {
    advertise<std_msgs::String>("foo");

    timer = new wxTimer(this);

    this->Connect( wxEVT_TIMER, wxTimerEventHandler( ImpLogGui::tick ), NULL, this );

    timer->Start(1000);

    wxTreeItemId root_id = topicTree->AddRoot(wxT("/"));
  }

  ~ImpLogGui() {}


  

  virtual void tick( wxTimerEvent& event )
  {
    topicList topics;

    get_published_topics(&topics);

    wxTreeItemId root_id = topicTree->GetRootItem();

    for (int pass = 0; pass < 2; pass++)
    {
      wxTreeItemId node = topicTree->GetRootItem();
      wxTreeItemIdValue cookie;

      while (node.IsOk()) {
        if (topicTree->ItemHasChildren(node))
          node = topicTree->GetFirstChild(node, cookie);
        else
        {
          bool needsDeleting = false;

          treeData* data = (treeData*)topicTree->GetItemData(node);

          if (data == NULL)
            needsDeleting = true;
          else
          {
            switch (pass) {
            case 0:
              data->save = false;
              for (topicList::iterator i = topics.begin(); i != topics.end(); i++)
              {
                if ( topicTree->GetItemText(node) == wxString::FromAscii( (i->first).c_str() ) )
                {
                  data->save = true;
                  topics.erase(i);
                  break;
                }
              }
              break;
            case 1:
              if (data->save == false)
                needsDeleting = true;
              break;
            }
          }
          
          wxTreeItemId tmp = node;

          wxTreeItemId n_node = topicTree->GetNextSibling(node);
          if (n_node.IsOk())
            node = n_node;
          else
          {
            wxTreeItemId p_node = topicTree->GetItemParent(node);
            if (p_node.IsOk())
              node = topicTree->GetNextSibling(p_node);
            else
              node = p_node;
          }
          
          if (needsDeleting && tmp != root_id)
            topicTree->Delete(tmp);
        }
      }
    }

    for (topicList::iterator i = topics.begin(); i != topics.end(); i++)
    {
      wxTreeItemId id = topicTree->AppendItem(root_id, wxString::FromAscii( (i->first).c_str() ));
      treeData* data = new treeData();
      topicTree->SetItemData(id, data);
    }

    
    Refresh();
  }
  

  virtual void openDir( wxCommandEvent& event ) 
  {
    wxString dir = wxDirSelector( wxT("Choose a folder") );
    if ( !dir.empty() )
    {
      printf("Tried to open %s\n", (const char*)dir.mb_str(wxConvUTF8));
      dir += wxT("/baz");
      if (mkdir((const char*)dir.mb_str(wxConvUTF8), 0755))
        printf("failed to make directory!\n");
    }
    event.Skip();
  }
  
  virtual void onClose( wxCloseEvent& event )
  {
    printf("Shutting down ros...\n");
    ros::fini();
    
    event.Skip();
  }

};

class LogGuiApp : public wxApp
{
public:
  LogGuiApp()
  {
    int argc = 0;
    ros::init(argc, NULL);
  };
  virtual ~LogGuiApp() { };
  virtual bool OnInit()
  {
    ImpLogGui* l = new ImpLogGui( (wxWindow*)NULL );
    l->Show();
    SetTopWindow( l );
    return true;
  }
};

DECLARE_APP(LogGuiApp);

IMPLEMENT_APP(LogGuiApp);


