import os
from slam_server import SlamServer

def main():
    server = SlamServer("0.0.0.0", 7000)
    server.setup_routes()
    server.run()

if __name__ == "__main__":
    main()
