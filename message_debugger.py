"""Observe real robot deliveries and display a horizontal message timeline."""

from collections import deque
from copy import deepcopy
from dataclasses import dataclass

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrowPatch
from matplotlib.ticker import MaxNLocator
from matplotlib.widgets import Button, Slider


@dataclass
class Delivery:
    number: int
    sender: int
    recipient: int
    sent_at: int
    expected_at: int
    payload: dict
    payload_identity: int
    arrived_at: int = None
    status: str = "in flight"
    receiver_state: str = None
    receiver_value: int = None

    @property
    def start(self):
        # Sending occurs after receiving within each iteration.
        return self.sent_at + 0.72

    @property
    def end(self):
        return (self.arrived_at if self.arrived_at is not None else self.expected_at) + 0.18


class MessageTrace:
    """Bounded, passive trace with a distinct record for each broadcast recipient."""

    def __init__(self, capacity=2000):
        if capacity < 1:
            raise ValueError("Trace capacity must be positive.")
        self.deliveries = deque(maxlen=capacity)
        self.pending = {}
        self.total_sent = 0
        self.total_received = 0
        self.total_dropped = 0

    def attach(self, robots):
        for item in robots:
            item.message_trace = self

    def sent(self, sender, recipient, payload):
        if len(self.deliveries) == self.deliveries.maxlen:
            self.pending.pop(self.deliveries[0].payload_identity, None)
        self.total_sent += 1
        delivery = Delivery(
            self.total_sent, sender.id, recipient.id, sender.iteration,
            # Even a zero-delay send is first handled on the next iteration.
            sender.iteration + int(payload["delay"]) + 1,
            deepcopy(payload), id(payload),
        )
        self.deliveries.append(delivery)
        self.pending[id(payload)] = delivery

    def arrived(self, recipient, payload, dropped=False):
        if dropped:
            self.total_dropped += 1
        else:
            self.total_received += 1
        delivery = self.pending.pop(id(payload), None)
        if delivery is not None:
            delivery.arrived_at = recipient.iteration
            delivery.status = "dropped" if dropped else "received"
            delivery.receiver_state = recipient.state
            delivery.receiver_value = recipient.message_value


class MessageDebugger:
    """Live simulation controls and a selectable, scrollable sequence diagram."""

    WINDOW = 12
    COLORS = ("#dc2626", "#0284c7", "#7c3aed", "#059669", "#d97706", "#db2777")

    def __init__(self, world, trace, draw_map, interval=1.0):
        self.world, self.trace, self.draw_map = world, trace, draw_map
        self.iteration = 0
        self.paused = False
        self.steps = 0
        self.follow = True
        self.window_end = self.WINDOW
        self.selected = None
        self.interval = interval
        self.finished = False
        self.closed = False
        self.fig = plt.figure(figsize=(16, 8.5))
        self.fig.canvas.manager.set_window_title("Robot simulation — message debugger")
        self.map_ax = self.fig.add_axes((0.045, 0.39, 0.30, 0.53))
        self.timeline_ax = self.fig.add_axes((0.46, 0.39, 0.51, 0.48))
        self.detail = self.fig.text(0.46, 0.29, "", va="top", fontsize=10, linespacing=1.55)
        self.state_text = self.fig.text(0.045, 0.29, "", va="top", fontsize=9, linespacing=1.65)
        self.status_text = self.fig.text(0.045, 0.965, "", fontsize=13, weight="bold")
        self.fig.text(0.46, 0.94, "MESSAGE TRAVEL  /  time flows to the right", fontsize=13, weight="bold")
        self.fig.text(0.46, 0.905, "Click an arrow to inspect its payload. Colors identify senders.", fontsize=10)
        self.buttons = []  # Matplotlib widgets must be kept alive.
        self.pause_button = self._button(0.045, "Pause", self.toggle_pause)
        self._button(0.145, "Step", self.step)
        self._button(0.46, "Earlier", self.earlier)
        self._button(0.56, "Later", self.later)
        self._button(0.66, "Live", self.live)
        self.speed = Slider(self.fig.add_axes((0.28, 0.065, 0.13, 0.025)),
                            "Seconds / step", 0.05, max(2.0, interval), valinit=interval, valfmt="%.2f")
        self.speed.on_changed(self._speed_changed)
        self.fig.text(0.045, 0.018, "Space: pause / resume    N: one iteration    Left / Right: history    L: live", fontsize=9)
        self.fig.canvas.mpl_connect("key_press_event", self._key)
        self.fig.canvas.mpl_connect("pick_event", self._pick)
        self.fig.canvas.mpl_connect("close_event", self._close)

    def _button(self, x, label, callback):
        button = Button(self.fig.add_axes((x, 0.105, 0.085, 0.045)), label)
        button.on_clicked(callback)
        self.buttons.append(button)
        return button

    def _speed_changed(self, value):
        self.interval = value

    def _close(self, event):
        self.closed = True

    def toggle_pause(self, event=None):
        if not self.finished:
            self.paused = not self.paused
            self.steps = 0
            if not self.paused:
                self.follow = True
        self.refresh()

    def step(self, event=None):
        if not self.finished:
            self.paused = True
            self.follow = True
            self.steps += 1
        self.refresh()

    def earlier(self, event=None):
        self.paused, self.follow = True, False
        self.steps = 0
        self.window_end = max(self.WINDOW, self.window_end - self.WINDOW // 2)
        self.refresh()

    def later(self, event=None):
        self.paused, self.follow = True, False
        self.steps = 0
        self.window_end = min(max(self.WINDOW, self.iteration + 2), self.window_end + self.WINDOW // 2)
        self.refresh()

    def live(self, event=None):
        self.follow = True
        self.refresh()

    def _key(self, event):
        action = {" ": self.toggle_pause, "n": self.step, "right": self.later,
                  "left": self.earlier, "l": self.live}.get(event.key)
        if action:
            action()

    def _pick(self, event):
        delivery = getattr(event.artist, "delivery", None)
        if delivery is not None:
            self.selected = delivery
            self.paused = True
            self.steps = 0
            self.refresh()

    def ready(self):
        if self.finished:
            return False
        if self.steps:
            self.steps -= 1
            return True
        return not self.paused

    def update(self, iteration):
        self.iteration = iteration
        self.draw_map(self.world, ax=self.map_ax)
        self.refresh()

    def refresh(self):
        if self.closed:
            return
        self.pause_button.label.set_text("Finished" if self.finished else "Resume" if self.paused else "Pause")
        mode = "Finished" if self.finished else "Paused" if self.paused else "Running"
        self.status_text.set_text(f"{mode}  |  Iteration {self.iteration}  |  Score {self.world.score}")
        robots = list(self.world.robots.values())
        self.state_text.set_text("CURRENT ROBOT STATES (map stays at the latest iteration)\n" + "\n".join(
            f"R{r.id}: {r.state}  |  n={r.message_value}  |  inbox={len(r.inbox)}" for r in robots))
        self._timeline(robots)
        self._details()
        self.fig.canvas.draw_idle()

    def _timeline(self, robots):
        ax = self.timeline_ax
        ax.clear()
        if self.follow:
            self.window_end = max(self.WINDOW, self.iteration + 2)
        left, right = self.window_end - self.WINDOW, self.window_end
        rows = {r.id: index for index, r in enumerate(robots)}
        colors = {r.id: self.COLORS[index % len(self.COLORS)] for index, r in enumerate(robots)}
        ax.set_xlim(left, right)
        ax.set_ylim(len(robots) - 0.5 if robots else 0.5, -0.75)
        ax.set_yticks(list(rows.values()), [f"Robot {r.id}" for r in robots])
        ax.xaxis.set_major_locator(MaxNLocator(integer=True))
        ax.set_xlabel("Simulation iteration (send late in step; receive early in step)", fontsize=9)
        ax.grid(axis="x", color="#e2e8f0", linewidth=0.6)
        for spine in ax.spines.values():
            spine.set_visible(False)
        for row in rows.values():
            ax.annotate("", (right, row), (left, row),
                        arrowprops={"arrowstyle": "->", "color": "#94a3b8", "lw": 0.8})
        now = self.iteration + 0.95
        ax.axvline(now, color="#64748b", linewidth=1, linestyle=":")
        visible = [d for d in self.trace.deliveries
                   if d.start <= right and d.end >= left and d.sender in rows and d.recipient in rows]
        # One caption per broadcast; each recipient still has its own selectable arrow.
        labeled = set()
        for delivery in visible:
            start, end = delivery.start, delivery.end
            y1, y2 = rows[delivery.sender], rows[delivery.recipient]
            color = colors[delivery.sender]
            arrow = FancyArrowPatch((start, y1), (end, y2), arrowstyle="-|>",
                                    mutation_scale=12, color=color,
                                    linewidth=2.5 if delivery is self.selected else 1.3,
                                    linestyle="--" if delivery.status == "in flight" else "-",
                                    alpha=0.85, picker=6, zorder=3)
            arrow.delivery = delivery
            ax.add_patch(arrow)
            ax.plot(start, y1, "o", color=color, markersize=4)
            if delivery.status == "in flight":
                progress = max(0, min(1, (now - start) / (end - start)))
                ax.plot(start + progress * (end - start), y1 + progress * (y2 - y1),
                        "o", color=color, markersize=7, markeredgecolor="white", zorder=5)
            elif delivery.status == "dropped":
                ax.plot(end, y2, "x", color=color, markersize=9, markeredgewidth=2, zorder=5)
            else:
                ax.plot(end, y2, "o", color=color, markersize=4, zorder=5)
            key = (delivery.sender, delivery.sent_at, delivery.payload["message_value"])
            if key not in labeled and left <= start <= right:
                labeled.add(key)
                ax.text(start + 0.05, y1 - 0.10,
                        f'{delivery.payload["message"]}\n[n={delivery.payload["message_value"]}]',
                        color=color, fontsize=8, va="bottom", clip_on=True,
                        bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.8, "pad": 1})
        if not visible:
            ax.text(0.5, 0.5, "No messages in this time window.\nRun or step until a robot requests help.",
                    transform=ax.transAxes, ha="center", color="#64748b")
        ax.legend(handles=[Line2D([], [], color="#475569", linestyle="--", marker="o", label="In flight / expected arrival"),
                           Line2D([], [], color="#475569", marker=">", label="Received"),
                           Line2D([], [], color="#475569", marker="x", label="Dropped (stale n)")],
                  loc="upper left", bbox_to_anchor=(0, 1.04), ncol=3, fontsize=8, frameon=False)

    def _details(self):
        d = self.selected
        if d is None:
            self.detail.set_text(
                "MESSAGE INSPECTOR\nSelect an arrow to see the payload, delay and receiver state.\n"
                f"Recorded: {self.trace.total_sent} sends, {self.trace.total_received} received, "
                f"{self.trace.total_dropped} dropped. Keeping the latest {self.trace.deliveries.maxlen} deliveries.\n"
                "Dashed arrows show scheduled travel; the dot shows progress through the delay.\n"
                "Received means passed the version check; a robot may take no action in its current state.")
            return
        p = d.payload
        arrival = f"actual {d.arrived_at}" if d.arrived_at is not None else f"expected {d.expected_at}"
        outcome = f"Receiver before delivery: {d.receiver_state}, n={d.receiver_value}" if d.arrived_at is not None else "Waiting in the recipient's inbox."
        if d.status == "dropped":
            outcome += f' — rejected because {p["message_value"]} < {d.receiver_value}'
        self.detail.set_text(
            f"MESSAGE #{d.number}   Robot {d.sender} → Robot {d.recipient}   [{d.status}]\n"
            f'{p["message"]}   n={p["message_value"]}   goal={p.get("goal")}   helper={p.get("helper_robot_id")}\n'
            f'Sent: {d.sent_at}   Arrival: {arrival}   Delay: {p["delay"]} countdown steps + next receive phase\n'
            f'Position at send: {p["position"]}   Distance to goal: {p.get("distance_to_goal")}\n'
            f"{outcome}")
