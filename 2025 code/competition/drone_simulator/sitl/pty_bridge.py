"""Expose a SITL TCP serial port as a pseudo-terminal, so Link sees a "serial cable".

Killing this bridge (Ctrl-C) behaves like unplugging the Cube's USB/UART:
reads on the pty fail with an error, which is what triggers Link's reconnect.
(A plain TCP link does not do that: pymavlink treats TCP EOF as "no data".)

    python3 -m competition.drone_simulator.sitl.pty_bridge --tcp 127.0.0.1:5762 --link /tmp/sitl-serial

Then point Link at ``/tmp/sitl-serial``. Restart the bridge to "plug the cable back in".
"""

import argparse
import logging
import os
import pty
import select
import signal
import socket
import sys
import time
import tty

logger = logging.getLogger("pty_bridge")


def connect_tcp(host: str, port: int) -> socket.socket:
    while True:
        try:
            sock = socket.create_connection((host, port), timeout=2.0)
            sock.settimeout(None)  # select() decides when to read
            logger.info("Connected to SITL at %s:%d", host, port)
            return sock
        except OSError as exc:
            logger.info("Waiting for SITL at %s:%d (%s)", host, port, exc)
            time.sleep(1.0)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--tcp", default="127.0.0.1:5762", help="SITL host:port (default SERIAL1)")
    parser.add_argument("--link", default="/tmp/sitl-serial", help="symlink to create for the pty")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s: %(message)s",
                        datefmt="%H:%M:%S")
    host, port = args.tcp.rsplit(":", 1)

    # SIGTERM -> normal exit so the symlink is cleaned up.
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))

    master_fd, slave_fd = pty.openpty()
    tty.setraw(slave_fd)  # binary-safe: no echo, no line editing
    slave_name = os.ttyname(slave_fd)
    sock = connect_tcp(host, int(port))
    if os.path.lexists(args.link):
        os.remove(args.link)
    os.symlink(slave_name, args.link)
    logger.info("Serving %s -> %s. Ctrl-C to 'unplug'.", args.link, slave_name)

    try:
        while True:
            readable, _, _ = select.select([master_fd, sock], [], [], 1.0)
            if master_fd in readable:
                sock.sendall(os.read(master_fd, 4096))
            if sock in readable:
                data = sock.recv(4096)
                if not data:
                    logger.error("SITL closed the TCP connection")
                    return 1
                os.write(master_fd, data)
    except KeyboardInterrupt:
        logger.info("Unplugged")
        return 0
    finally:
        if os.path.islink(args.link) and os.readlink(args.link) == slave_name:
            os.remove(args.link)
        sock.close()
        os.close(master_fd)
        os.close(slave_fd)


if __name__ == "__main__":
    sys.exit(main())
