"""
Reflex Smash - Client
Connect to the server and press Enter when you see GO!

Usage:
  python client.py                  # connects to localhost
  python client.py 192.168.1.42     # connects to a remote server
"""

import socket
import threading
import sys


def receive(sock):
    """Print everything the server sends."""
    f = sock.makefile("r")
    while True:
        line = f.readline()
        if not line:
            print("\n[disconnected]")
            return
        print(line.rstrip())


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 5555

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((host, port))
    except ConnectionRefusedError:
        print(f"Could not connect to {host}:{port}. Is the server running?")
        return

    print(f"Connected to {host}:{port}\n")

    # Background thread reads from server
    threading.Thread(target=receive, args=(sock,), daemon=True).start()

    # Main thread reads keyboard. Any Enter press = a move.
    try:
        while True:
            input()  # waits for Enter
            try:
                sock.sendall(b"!\n")
            except Exception:
                break
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        sock.close()


if __name__ == "__main__":
    main()


