#!/usr/bin/env python3
import http.server
import socketserver
import os
import argparse
port = 8080

def run_server(host="0.0.0.0", port=port, directory="."):
    """open the visualizer.html file"""
    
    # change to the specified directory
    os.chdir(directory)
    
    # set the server handler
    handler = http.server.SimpleHTTPRequestHandler
    
    # create the server
    with socketserver.TCPServer((host, port), handler) as httpd:
        print(f"The server has been started, bound to {host}:{port}")
        print(f"Visit: http://{host}:{port}/visualizer.html")
        if host == "0.0.0.0":
            print(f"Or access from localhost: http://localhost:{port}/visualizer.html")
        print("Press Ctrl+C to stop the server")
        
        # start the server, until manually interrupted
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nThe server has been stopped")

if __name__ == "__main__":
    # create the command line argument parser
    parser = argparse.ArgumentParser(description="Start a simple HTTP server to display Floorplan visualization")
    parser.add_argument("--host", default="0.0.0.0", help="Server host address (default: 0.0.0.0, allows remote access)")
    parser.add_argument("--port", type=int, default=port, help="Server port number (default: 8000)")
    parser.add_argument("--dir", default=".", help="The directory to provide services (default: current directory)")
    
    # parse the command line arguments
    args = parser.parse_args()
    
    # run the server
    run_server(args.host, args.port, args.dir) 