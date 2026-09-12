"""Plot the reduced trajectory printed by ProcessingState or the host CLI."""

from __future__ import annotations

import argparse
import re
from dataclasses import dataclass
from pathlib import Path


SERIAL_HEADER = "trajectory_index,horizontal_mm,vertical_mm,turnaround"
SERIAL_POINT_RE = re.compile(
    r"(?:^|\s)(?P<index>\d+),(?P<horizontal>-?\d+),"
    r"(?P<vertical>-?\d+),(?P<turnaround>[01])\s*$"
)
CLI_POINT_RE = re.compile(
    r"(?:^|\s)point=(?P<index>\d+),(?P<horizontal>-?\d+),"
    r"(?P<vertical>-?\d+)\s*$"
)
CLI_TURNAROUND_RE = re.compile(r"turnaround_trajectory_index=(?P<index>\d+)")
ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")


@dataclass(frozen=True)
class TrajectoryPoint:
    index: int
    horizontal_mm: int
    vertical_mm: int
    turnaround: bool


def parse_trajectories(log_text: str) -> list[list[TrajectoryPoint]]:
    """Return every complete trajectory block found in a serial/CLI log."""
    trajectories: list[list[TrajectoryPoint]] = []
    current: list[TrajectoryPoint] | None = None
    cli_turnaround_index: int | None = None

    for raw_line in log_text.splitlines():
        line = ANSI_ESCAPE_RE.sub("", raw_line).strip()
        if CLI_TURNAROUND_RE.search(line):
            cli_turnaround_index = int(CLI_TURNAROUND_RE.search(line).group("index"))

        if SERIAL_HEADER in line:
            if current:
                trajectories.append(current)
            current = []
            continue

        match = CLI_POINT_RE.search(line)
        is_cli_point = match is not None
        if match is None and current is not None:
            match = SERIAL_POINT_RE.search(line)
        if match is None:
            continue
        if current is None:
            current = []

        index = int(match.group("index"))
        turnaround = (
            index == cli_turnaround_index
            if is_cli_point
            else match.group("turnaround") == "1"
        )
        current.append(
            TrajectoryPoint(
                index=index,
                horizontal_mm=int(match.group("horizontal")),
                vertical_mm=int(match.group("vertical")),
                turnaround=turnaround,
            )
        )

    if current:
        trajectories.append(current)
    return trajectories


def validate(points: list[TrajectoryPoint]) -> None:
    """Reject truncated or ambiguous trajectory blocks before plotting."""
    if len(points) < 3:
        raise ValueError("Znaleziono mniej niż trzy punkty trajektorii.")
    indices = [point.index for point in points]
    if indices != list(range(len(points))):
        raise ValueError(
            "Indeksy trajektorii nie są ciągłe od zera; log może być ucięty."
        )
    if sum(point.turnaround for point in points) != 1:
        raise ValueError("Nie znaleziono dokładnie jednego oznaczonego punktu zwrotnego.")


def plot(points: list[TrajectoryPoint], output_path: Path, show: bool) -> None:
    """Render one horizontal/vertical barbell trajectory and save it as PNG."""
    try:
        import matplotlib.pyplot as plt
    except ModuleNotFoundError as error:
        raise SystemExit(
            "Brak matplotlib. Uruchom: pip install -r src/utils/requirements.txt"
        ) from error

    horizontal_cm = [point.horizontal_mm / 10.0 for point in points]
    vertical_cm = [point.vertical_mm / 10.0 for point in points]
    turnaround = next(i for i, point in enumerate(points) if point.turnaround)

    figure, axis = plt.subplots(figsize=(7.5, 6.0), constrained_layout=True)
    axis.plot(
        horizontal_cm[: turnaround + 1],
        vertical_cm[: turnaround + 1],
        color="#1f77b4",
        linewidth=2.5,
        label="Opuszczanie",
    )
    axis.plot(
        horizontal_cm[turnaround:],
        vertical_cm[turnaround:],
        color="#2ca02c",
        linewidth=2.5,
        label="Wyciskanie",
    )
    axis.scatter(
        [horizontal_cm[0], horizontal_cm[turnaround], horizontal_cm[-1]],
        [vertical_cm[0], vertical_cm[turnaround], vertical_cm[-1]],
        color=["black", "#d62728", "black"],
        s=[45, 80, 45],
        zorder=3,
    )
    for label, index, offset in (
        ("start", 0, (6, 6)),
        ("punkt zwrotny", turnaround, (6, -16)),
        ("koniec", len(points) - 1, (6, 6)),
    ):
        axis.annotate(
            label,
            (horizontal_cm[index], vertical_cm[index]),
            xytext=offset,
            textcoords="offset points",
        )

    axis.axhline(0.0, color="0.7", linewidth=0.8)
    axis.set_title("Trajektoria sztangi obliczona w firmware")
    axis.set_xlabel("Poziomo, kierunek PCA [cm]")
    axis.set_ylabel("Pion, Z w górę [cm]")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(alpha=0.3)
    axis.legend()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=180, bbox_inches="tight")
    print(f"Zapisano wykres: {output_path.resolve()}")
    if show:
        plt.show()
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Rysuje ostatnią trajektorię znalezioną w logu firmware."
    )
    parser.add_argument("log", type=Path, help="Plik zapisany z Serial Monitora")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("firmware_trajectory.png"),
        help="Docelowy plik PNG (domyślnie: firmware_trajectory.png)",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Tylko zapisz PNG, bez otwierania okna z wykresem",
    )
    args = parser.parse_args()

    trajectories = parse_trajectories(args.log.read_text(encoding="utf-8"))
    if not trajectories:
        raise ValueError(
            f"Brak danych po nagłówku '{SERIAL_HEADER}' w pliku {args.log}."
        )
    points = trajectories[-1]
    validate(points)
    plot(points, args.output, show=not args.no_show)


if __name__ == "__main__":
    main()
