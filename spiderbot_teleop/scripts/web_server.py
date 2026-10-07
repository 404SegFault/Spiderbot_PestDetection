#!/usr/bin/env python
"""Serve the teleop page (www/) over HTTP, so a phone on the same network can open it.

The page itself talks to rosbridge; this node only hands out the files.
"""
import os
import threading

import rospy

try:  # Python 2 (Melodic)
    from BaseHTTPServer import HTTPServer
    from SimpleHTTPServer import SimpleHTTPRequestHandler
    from SocketServer import ThreadingMixIn
except ImportError:  # Python 3
    from http.server import HTTPServer, SimpleHTTPRequestHandler
    from socketserver import ThreadingMixIn


class QuietHandler(SimpleHTTPRequestHandler):
    def log_message(self, *args):
        pass


class ThreadingServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main():
    rospy.init_node("web_teleop_server")
    port = rospy.get_param("~port", 8080)
    os.chdir(rospy.get_param("~www"))  # the page folder, set by the launch file
    server = ThreadingServer(("", port), QuietHandler)
    rospy.on_shutdown(server.shutdown)
    thread = threading.Thread(target=server.serve_forever)
    thread.daemon = True
    thread.start()
    rospy.loginfo("Teleop page: http://<this machine's IP>:%d/ (needs rosbridge on port 9090)", port)
    rospy.spin()


if __name__ == "__main__":
    main()
