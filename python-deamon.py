import http.server

class MyHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(s):
        """Respond to a GET request."""
        s.send_response(403)
        print("\a\a\agot SOS!!!\n"); 

def run(server_class=http.server.HTTPServer, handler_class=MyHandler):
    server_address = ('192.168.1.236', 80)
    httpd = server_class(server_address, handler_class)
    httpd.serve_forever()

run()
