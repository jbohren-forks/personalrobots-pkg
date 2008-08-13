#!/usr/bin/env python
from pkg import *
from std_msgs.msg import String
import wx, time, sys

def key_to_command(key):
    ret = None

    if key == u'g':
        ret = 'debug'

    if key == u'd':
        ret = 'display'

    if key == u'v':
        ret = 'verbose'

    if key == u' ':
        ret = 'rebuild'

    if key == u'p':
        ret = 'positive'

    if key == u'c':
        ret = 'clear'

    return ret

class KeyHandler(wx.Window):
    def __init__(self, parent):
        wx.Window.__init__(self, parent, -1, style=wx.WANTS_CHARS, name="sink")
        self.Bind(wx.EVT_KEY_DOWN, self.on_key_down)
        self.Bind(wx.EVT_LEFT_DOWN, self.on_left_down)
        self.Bind(wx.EVT_LEFT_UP, self.on_left_up)
        self.Bind(wx.EVT_PAINT, self.on_paint)
        self.Bind(wx.EVT_SET_FOCUS, self.on_set_focus)
        self.Bind(wx.EVT_KILL_FOCUS, self.on_kill_focus)
        self.Bind(wx.EVT_SIZE, self.on_size)
        self.Bind(wx.EVT_UPDATE_UI, self.on_size)
        wx.UpdateUIEvent.SetUpdateInterval(500)

        self.mouse_click_pub = rospy.TopicPub(MOUSE_CLICK_TOPIC, String).publish
        self.laser_mode_pub  = rospy.TopicPub(LASER_MODE_TOPIC, String).publish
        self.SetBackgroundColour(wx.BLACK)
        self.focus = False
        self.text = ''
        self.time = time.time()
        self.interval = 1.0

    def set_text(self, text):
        self.text = text
        self.time = time.time()

    def on_size(self, evt):
        self.Refresh()

    def on_set_focus(self, evt):
        self.focus = True
        self.Refresh()

    def on_kill_focus(self, evt):
        self.focus = False
        self.Refresh()

    def on_left_down(self, evt):
        self.mouse_click_pub(String('True'))
        self.set_text('cli....')
        self.Refresh()

    def on_left_up(self, evt):
        self.mouse_click_pub(String('False'))
        self.set_text('...cked')
        self.Refresh()

    def on_key_down(self, evt):
        c = unichr(evt.GetRawKeyCode())
        command = key_to_command(c)
        if command is not None:
            self.laser_mode_pub(String(command))
            self.set_text(command)
        else:
            self.set_text(unichr(evt.GetRawKeyCode()))
        self.Refresh()

    def on_paint(self, evt):
        dc   = wx.PaintDC(self)
        font = dc.GetFont()
        font.SetPointSize(20)
        dc.SetFont(font)
        rect = self.GetClientRect()

        if self.focus:
            dc.SetTextForeground(wx.GREEN)
            dc.DrawLabel("Focused.", rect, wx.ALIGN_CENTER)
        else:
            dc.SetTextForeground(wx.RED)
            dc.DrawLabel("Got no focus.", rect, wx.ALIGN_CENTER)

        dc.SetTextForeground(wx.WHITE)
        dc.DrawLabel('g - debug\nd - display\nv - verbose\np - positive\nc - clear\nspace - rebuild', rect, wx.ALIGN_LEFT | wx.ALIGN_TOP)

        if (time.time() - self.time) < self.interval:
            dc.SetFont(wx.Font(20, wx.MODERN, wx.NORMAL, wx.NORMAL))
            dc.SetTextForeground(wx.WHITE)
            dc.DrawLabel(self.text, rect, wx.ALIGN_BOTTOM | wx.ALIGN_CENTER)

app   = wx.PySimpleApp()
frame = wx.Frame(None, wx.ID_ANY, name='Clickable World GUI', size=(800,600))
win   = KeyHandler(frame) 
frame.Show(True)
win.SetFocus()
rospy.ready(sys.argv[0])
app.MainLoop()


