//-----------------------------------------------------------------------------
std::string wx2std(const wxString& input, wxMBConv* conv = 0)
{
     if (input.empty())
         return "";
     if (!conv)
         conv = wxConvCurrent;
     const wxWX2MBbuf buf(input.mb_str(*conv));
     // conversion may fail and return 0,
     // which isn't a safe value to pass
     // to std:string constructor
     if (!buf)
         return "";
     return std::string(buf);
}
//-----------------------------------------------------------------------------
wxString std2wx(const std::string& input, wxMBConv* conv = 0)
{
    if (input.empty())
        return wxEmptyString;
    if (!conv)
        conv = wxConvCurrent;
    return wxString(input.c_str(), *conv);
}
