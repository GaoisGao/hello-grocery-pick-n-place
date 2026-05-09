"""
Reflex Smash - Server
A 2-player reflex game over sockets.

Each round:
  - Server waits a random 2-5 seconds, then sends "GO!" to both players.
  - First player to press Enter wins the round.
  - Pressing BEFORE the GO is a false start (you lose the round).

Best of 5. After the game ends, the server posts a leaderboard.
"""

import socket
import threading
import time
import random

# --- Settings ---
HOST = "0.0.0.0"
PORT = 5555
NUM_PLAYERS = 2
ROUNDS = 5
MIN_WAIT = 2.0      # min seconds before GO
MAX_WAIT = 5.0      # max seconds before GO

# --- Shared state ---
lock = threading.Lock()
players = {}        # pid -> {"conn", "wins", "best_ms"}
go_time = None      # timestamp of the most recent GO! (set per round)
round_active = False
# pid -> reaction time in ms for the current round (or "false_start")
round_results = {}


def send(conn, text):
    try:
        conn.sendall((text + "\n").encode())
    except Exception:
        pass


def broadcast(text):
    for p in players.values():
        send(p["conn"], text)


def handle_input(conn, pid):
    """Listen for keypresses (any line) from one player."""
    global round_results
    f = conn.makefile("r")
    while True:
        line = f.readline()
        if not line:
            break
        # Mark the moment we received their press
        press_time = time.time()
        with lock:
            if not round_active:
                # Game isn't accepting input right now — ignore
                continue
            if pid in round_results:
                continue  # already pressed this round
            if go_time is None:
                # Pressed during the wait phase = false start
                round_results[pid] = "false_start"
            else:
                ms = int((press_time - go_time) * 1000)
                round_results[pid] = ms
    # On disconnect, mark the player out
    with lock:
        if pid in players:
            players[pid]["disconnected"] = True


def play_round(round_num):
    """Run a single round and return the winning pid (or None)."""
    global go_time, round_active, round_results

    with lock:
        round_results = {}
        go_time = None
        round_active = True

    broadcast(f"\n--- Round {round_num} of {ROUNDS} ---")
    broadcast("Get ready... wait for GO! then press Enter.")

    # Random wait, but check periodically for false starts
    wait = random.uniform(MIN_WAIT, MAX_WAIT)
    deadline = time.time() + wait
    while time.time() < deadline:
        time.sleep(0.05)
        with lock:
            # If anyone false-started, end the round early
            if any(v == "false_start" for v in round_results.values()):
                break

    with lock:
        # Check for a false start before we send GO
        false_starters = [pid for pid, v in round_results.items()
                          if v == "false_start"]
        if false_starters:
            round_active = False
            for pid in false_starters:
                broadcast(f"Player {pid + 1} FALSE STARTED!")
            # Other player wins the round
            others = [pid for pid in players if pid not in false_starters]
            if len(others) == 1:
                winner = others[0]
                players[winner]["wins"] += 1
                broadcast(f"Player {winner + 1} wins round {round_num}.")
                return winner
            return None

        # Send GO simultaneously to all players
        broadcast(">>> GO! <<<")
        go_time = time.time()

    # Wait up to 3 seconds for both players to respond
    end = time.time() + 3.0
    while time.time() < end:
        time.sleep(0.02)
        with lock:
            if len(round_results) >= NUM_PLAYERS:
                break

    with lock:
        round_active = False

        # Anyone who didn't respond gets a "miss"
        for pid in players:
            if pid not in round_results:
                round_results[pid] = "miss"

        # Find the winner: lowest valid reaction time
        valid = {pid: ms for pid, ms in round_results.items()
                 if isinstance(ms, int)}

        # Report each player's result
        for pid in sorted(players.keys()):
            r = round_results[pid]
            if r == "false_start":
                broadcast(f"Player {pid + 1}: FALSE START")
            elif r == "miss":
                broadcast(f"Player {pid + 1}: too slow (no response)")
            else:
                broadcast(f"Player {pid + 1}: {r} ms")

        if not valid:
            broadcast("Nobody scored this round.")
            return None

        winner = min(valid, key=valid.get)
        players[winner]["wins"] += 1
        # Track best reaction time
        if (players[winner]["best_ms"] is None
                or valid[winner] < players[winner]["best_ms"]):
            players[winner]["best_ms"] = valid[winner]

        broadcast(f"Player {winner + 1} wins round {round_num}!")
        return winner


def run_game():
    """Wait for players, then run the full match."""
    # Wait for everyone to connect
    while True:
        with lock:
            if len(players) >= NUM_PLAYERS:
                break
        time.sleep(0.2)

    broadcast("\n=== REFLEX SMASH ===")
    broadcast(f"Best of {ROUNDS} rounds. First press wins each round.")
    broadcast("Don't press before GO! or you lose the round.\n")
    time.sleep(2)

    for r in range(1, ROUNDS + 1):
        play_round(r)
        time.sleep(1.5)

    # Final leaderboard
    broadcast("\n=== FINAL SCOREBOARD ===")
    ranking = sorted(players.items(),
                     key=lambda kv: (-kv[1]["wins"],
                                     kv[1]["best_ms"] or 99999))
    for rank, (pid, data) in enumerate(ranking, start=1):
        best = f"{data['best_ms']} ms" if data["best_ms"] is not None else "—"
        broadcast(f"  {rank}. Player {pid + 1}  —  "
                  f"{data['wins']} wins, best: {best}")

    top_wins = ranking[0][1]["wins"]
    champs = [pid for pid, d in ranking if d["wins"] == top_wins]
    if len(champs) == 1:
        broadcast(f"\n*** Player {champs[0] + 1} is the CHAMPION! ***\n")
    else:
        names = ", ".join(f"Player {p + 1}" for p in champs)
        broadcast(f"\n*** Tie between {names}! ***\n")


def main():
    print(f"[server] listening on port {PORT}")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen(NUM_PLAYERS)

    threading.Thread(target=run_game, daemon=True).start()

    pid = 0
    while pid < NUM_PLAYERS:
        conn, addr = s.accept()
        with lock:
            players[pid] = {"conn": conn, "wins": 0, "best_ms": None}
        send(conn, f"Welcome! You are Player {pid + 1}.")
        send(conn, f"Waiting for {NUM_PLAYERS - pid - 1} more player(s)...")
        print(f"[server] player {pid + 1} connected from {addr}")
        threading.Thread(target=handle_input, args=(conn, pid),
                         daemon=True).start()
        pid += 1

    # Keep the server alive long enough for the game to finish
    time.sleep(ROUNDS * (MAX_WAIT + 5) + 5)
    s.close()
    print("[server] shutdown")


if __name__ == "__main__":
    main()


