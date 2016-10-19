#!/usr/bin/env python

import sys
import wx
import gvfformation

class MessagesApp(wx.App):
    def __init__(self, wtf, id_leader, id_follower, desired_sigma):
        self.l = id_leader
        self.f = id_follower
        self.sig = desired_sigma
        wx.App.__init__(self, wtf)

    def OnInit(self):
        self.main = gvfformation.GVFFrame(self.l, self.f, self.sig)
        self.main.Show()
        self.SetTopWindow(self.main)
        return True

def main():
    if len(sys.argv) != 4:
        print "Usage: gvfFormationApp id_leader id_follower desired_sigma"
        return
    id_leader = int(sys.argv[1])
    id_follower = int(sys.argv[2])
    desired_sigma = float(sys.argv[3])
    application = MessagesApp(0, id_leader, id_follower, desired_sigma)
    application.MainLoop()

if __name__ == '__main__':
    main()
