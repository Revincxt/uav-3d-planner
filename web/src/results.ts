import "./results.css";

import { loadBenchmarkBundle } from "./benchmark-data";
import type {
  BenchmarkBundleV2,
  DistributionSummary,
  PlannerBudget,
  PlannerId,
} from "./benchmark-schema";

const SVG_NS = "http://www.w3.org/2000/svg";
const DEFAULT_CHART_WIDTH = 680;

interface ChartPoint {
  x: number;
  metric: DistributionSummary;
}

interface ChartSeries {
  plannerId: PlannerId;
  label: string;
  points: ChartPoint[];
}

interface ChartOptions {
  title: string;
  description: string;
  xLabel: string;
  xScale: "linear" | "log";
  formatX: (value: number) => string;
  series: ChartSeries[];
}

const element = <T extends HTMLElement>(selector: string): T => {
  const value = document.querySelector<T>(selector);
  if (!value) throw new Error(`Missing required element: ${selector}`);
  return value;
};

function svgElement(tag: string, attributes: Record<string, string | number> = {}): SVGElement {
  const value = document.createElementNS(SVG_NS, tag);
  Object.entries(attributes).forEach(([name, attribute]) => value.setAttribute(name, String(attribute)));
  return value;
}

function svgText(
  x: number,
  y: number,
  value: string,
  className: string,
  anchor: "start" | "middle" | "end" = "start",
): SVGElement {
  const node = svgElement("text", { x, y, class: className, "text-anchor": anchor });
  node.textContent = value;
  return node;
}

function linearScale(domainMin: number, domainMax: number, rangeMin: number, rangeMax: number) {
  const span = domainMax - domainMin || 1;
  return (value: number): number => rangeMin + ((value - domainMin) / span) * (rangeMax - rangeMin);
}

function logScale(domainMin: number, domainMax: number, rangeMin: number, rangeMax: number) {
  const low = Math.log(domainMin);
  const span = Math.log(domainMax) - low || 1;
  return (value: number): number =>
    rangeMin + ((Math.log(value) - low) / span) * (rangeMax - rangeMin);
}

function axisNumber(value: number): string {
  const absolute = Math.abs(value);
  if (absolute >= 1000) return new Intl.NumberFormat("en", { maximumFractionDigits: 0 }).format(value);
  if (absolute >= 10) return value.toFixed(0);
  return value.toFixed(1).replace(/\.0$/, "");
}

function compactInteger(value: number): string {
  if (value >= 1000 && value % 1000 === 0) return `${value / 1000}k`;
  return new Intl.NumberFormat("en", { maximumFractionDigits: 0 }).format(value);
}

function selectedTicks(values: number[], maximum = 6): number[] {
  const unique = [...new Set(values)].sort((a, b) => a - b);
  if (unique.length <= maximum) return unique;
  const indices = new Set<number>([0, unique.length - 1]);
  for (let index = 1; index < maximum - 1; index += 1) {
    indices.add(Math.round((index * (unique.length - 1)) / (maximum - 1)));
  }
  return [...indices].sort((a, b) => a - b).map((index) => unique[index] as number);
}

function appendMarker(group: SVGElement, plannerId: PlannerId, x: number, y: number): void {
  let marker: SVGElement;
  if (plannerId === "astar-3d") {
    marker = svgElement("circle", { cx: x, cy: y, r: 5, class: "series-marker" });
  } else if (plannerId === "lazy-theta-star") {
    marker = svgElement("rect", {
      x: x - 4.5,
      y: y - 4.5,
      width: 9,
      height: 9,
      class: "series-marker",
    });
  } else {
    marker = svgElement("path", {
      d: `M ${x} ${y - 6} L ${x + 5.5} ${y + 4.5} L ${x - 5.5} ${y + 4.5} Z`,
      class: "series-marker",
    });
  }
  group.append(marker);
}

function renderSensitivityChart(host: HTMLElement, options: ChartOptions): void {
  host.replaceChildren();
  const measuredWidth = Math.round(host.getBoundingClientRect().width);
  const chartWidth = measuredWidth > 0 ? Math.max(280, measuredWidth) : DEFAULT_CHART_WIDTH;
  const compact = chartWidth < 480;
  const chartHeight = compact ? 300 : 340;
  const margin = compact
    ? { top: 52, right: 14, bottom: 58, left: 54 }
    : { top: 52, right: 28, bottom: 62, left: 72 };
  const svg = svgElement("svg", {
    class: "chart-svg",
    width: chartWidth,
    height: chartHeight,
    viewBox: `0 0 ${chartWidth} ${chartHeight}`,
    role: "img",
    "aria-labelledby": `${host.id}-title ${host.id}-description`,
  });
  const title = svgElement("title", { id: `${host.id}-title` });
  title.textContent = options.title;
  const description = svgElement("desc", { id: `${host.id}-description` });
  const pointDescriptions = options.series.flatMap((series) =>
    [...series.points]
      .sort((a, b) => a.x - b.x)
      .map((point) => {
        const metric = point.metric;
        const x = options.formatX(point.x);
        if (metric.median === null) {
          return `${series.label} at ${x}: no path-quality estimate; ${metric.nSuccesses} of ${metric.nRuns} runs succeeded.`;
        }
        return `${series.label} at ${x}: median ${axisNumber(metric.median)} percent, interquartile range ${axisNumber(metric.q1 as number)} to ${axisNumber(metric.q3 as number)} percent; ${metric.nSuccesses} of ${metric.nRuns} runs succeeded.`;
      }),
  );
  description.textContent = `${options.description} ${pointDescriptions.join(" ")}`;
  svg.append(title, description);

  const allPoints = options.series.flatMap((series) => series.points);
  const finitePoints = allPoints.filter(
    (point) => point.metric.median !== null && point.metric.q1 !== null && point.metric.q3 !== null,
  );
  if (finitePoints.length === 0) {
    svg.append(
      svgText(
        chartWidth / 2,
        chartHeight / 2,
        "No successful runs at the recorded settings.",
        "chart-empty",
        "middle",
      ),
    );
    host.append(svg);
    return;
  }

  const xValues = allPoints.map((point) => point.x);
  const xMin = Math.min(...xValues);
  const xMax = Math.max(...xValues);
  const qValues = finitePoints.flatMap((point) => [point.metric.q1 as number, point.metric.q3 as number]);
  const observedMax = Math.max(...qValues, 0);
  const yMin = Math.min(0, ...qValues);
  const yMax = observedMax === yMin ? observedMax + 1 : observedMax + (observedMax - yMin) * 0.12;
  const plotLeft = margin.left;
  const plotRight = chartWidth - margin.right;
  const plotTop = margin.top;
  const plotBottom = chartHeight - margin.bottom;
  const x =
    options.xScale === "log"
      ? logScale(xMin, xMax, plotLeft, plotRight)
      : linearScale(xMin, xMax, plotLeft, plotRight);
  const y = linearScale(yMin, yMax, plotBottom, plotTop);

  for (let index = 0; index <= 4; index += 1) {
    const value = yMin + ((yMax - yMin) * index) / 4;
    const position = y(value);
    svg.append(
      svgElement("line", {
        x1: plotLeft,
        x2: plotRight,
        y1: position,
        y2: position,
        class: "grid-line",
      }),
      svgText(plotLeft - 10, position + 4, axisNumber(value), "tick-label", "end"),
    );
  }

  svg.append(
    svgElement("line", {
      x1: plotLeft,
      x2: plotLeft,
      y1: plotTop,
      y2: plotBottom,
      class: "axis-line",
    }),
    svgElement("line", {
      x1: plotLeft,
      x2: plotRight,
      y1: plotBottom,
      y2: plotBottom,
      class: "axis-line",
    }),
  );

  selectedTicks(xValues, compact ? 4 : 6).forEach((value) => {
    const position = x(value);
    svg.append(
      svgElement("line", {
        x1: position,
        x2: position,
        y1: plotBottom,
        y2: plotBottom + 5,
        class: "tick-mark",
      }),
      svgText(position, plotBottom + 22, options.formatX(value), "tick-label", "middle"),
    );
  });

  svg.append(
    svgText((plotLeft + plotRight) / 2, chartHeight - 13, options.xLabel, "axis-label", "middle"),
  );
  const yLabel = svgText(18, (plotTop + plotBottom) / 2, "Raw-path excess (%)", "axis-label", "middle");
  yLabel.setAttribute("transform", `rotate(-90 18 ${(plotTop + plotBottom) / 2})`);
  svg.append(yLabel);

  if (options.series.length > 1) {
    let legendX = plotLeft;
    options.series.forEach((series) => {
      const group = svgElement("g", {
        class: `planner-${series.plannerId}`,
        "aria-hidden": "true",
      });
      group.append(
        svgElement("line", {
          x1: legendX,
          x2: legendX + 22,
          y1: 24,
          y2: 24,
          class: "series-line",
        }),
      );
      appendMarker(group, series.plannerId, legendX + 11, 24);
      group.append(svgText(legendX + 29, 28, series.label, "legend-label"));
      svg.append(group);
      legendX += series.label.length * 7 + 58;
    });
  }

  options.series.forEach((series, seriesIndex) => {
    const group = svgElement("g", { class: `planner-${series.plannerId}` });
    const ordered = [...series.points].sort((a, b) => a.x - b.x);
    let previous: { x: number; y: number } | null = null;
    ordered.forEach((point) => {
      const metric = point.metric;
      if (metric.median === null || metric.q1 === null || metric.q3 === null) {
        previous = null;
        return;
      }
      const px = x(point.x);
      const py = y(metric.median);
      if (previous) {
        group.append(
          svgElement("line", {
            x1: previous.x,
            y1: previous.y,
            x2: px,
            y2: py,
            class: "series-line",
          }),
        );
      }
      const q1 = y(metric.q1);
      const q3 = y(metric.q3);
      group.append(
        svgElement("line", { x1: px, x2: px, y1: q1, y2: q3, class: "series-iqr" }),
        svgElement("line", { x1: px - 4, x2: px + 4, y1: q1, y2: q1, class: "series-cap" }),
        svgElement("line", { x1: px - 4, x2: px + 4, y1: q3, y2: q3, class: "series-cap" }),
      );
      appendMarker(group, series.plannerId, px, py);
      const labelAnchor = options.series.length > 1 && seriesIndex === 0 ? "end" : "start";
      const labelX = labelAnchor === "end" ? px - 7 : px + 7;
      group.append(
        svgText(
          labelX,
          Math.max(plotTop + 12, q3 - 7),
          `${metric.nSuccesses}/${metric.nRuns}`,
          "success-label",
          labelAnchor,
        ),
      );
      previous = { x: px, y: py };
    });
    svg.append(group);
  });

  host.append(svg);
}

function mountSensitivityChart(host: HTMLElement, options: ChartOptions): void {
  let lastWidth = -1;
  let frame = 0;
  const render = (): void => {
    const width = Math.round(host.getBoundingClientRect().width);
    if (width === lastWidth) return;
    lastWidth = width;
    renderSensitivityChart(host, options);
  };
  const schedule = (): void => {
    window.cancelAnimationFrame(frame);
    frame = window.requestAnimationFrame(render);
  };
  render();
  if (typeof ResizeObserver !== "undefined") {
    new ResizeObserver(schedule).observe(host);
  } else {
    window.addEventListener("resize", schedule);
  }
}

function formatDistribution(metric: DistributionSummary, digits = 1): string {
  if (metric.median === null || metric.q1 === null || metric.q3 === null) return "—";
  const estimate = `${metric.median.toFixed(digits)} [${metric.q1.toFixed(digits)}, ${metric.q3.toFixed(digits)}]`;
  return metric.nDefinedScenes < metric.nScenes
    ? `${estimate} · ${metric.nDefinedScenes}/${metric.nScenes} defined scenes`
    : estimate;
}

function formatSuccess(value: BenchmarkBundleV2["summaries"][number]["successRate"]): string {
  return `${(value.value * 100).toFixed(1)} [${(value.ci95Low * 100).toFixed(1)}, ${(value.ci95High * 100).toFixed(1)}]`;
}

function budgetLabel(budget: PlannerBudget): string {
  const limit = budget.wallClockLimitMs === null ? "" : ` · ${compactInteger(budget.wallClockLimitMs)} ms cap`;
  if (budget.kind === "voxel") {
    return `${axisNumber(budget.voxelResolutionM)} m · ${compactInteger(budget.maxExpansions)} expansions${limit}`;
  }
  return `${compactInteger(budget.sampleBudget)} samples${limit}`;
}

function renderSummaryTable(bundle: BenchmarkBundleV2): void {
  const body = element<HTMLTableSectionElement>("#summary-body");
  const budgets = new Map(bundle.budgets.map((budget) => [budget.id, budget]));
  const summaries = new Map(bundle.summaries.map((summary) => [summary.plannerId, summary]));
  body.replaceChildren();
  bundle.planners.forEach((planner) => {
    const summary = summaries.get(planner.id);
    if (!summary) return;
    const budget = budgets.get(summary.budgetId);
    if (!budget) return;
    const row = document.createElement("tr");
    const heading = document.createElement("th");
    heading.scope = "row";
    const name = document.createElement("span");
    name.className = "planner-name";
    const symbol = document.createElement("i");
    symbol.className = `planner-symbol planner-${planner.id}`;
    symbol.setAttribute("aria-hidden", "true");
    name.append(symbol, document.createTextNode(planner.label));
    heading.append(name);
    row.append(heading);

    const values = [
      budgetLabel(budget),
      `${summary.successRate.nRuns.toLocaleString()} (${summary.successRate.nScenes.toLocaleString()})`,
      `${summary.planningTimeMs.nRuns.toLocaleString()} (${summary.planningTimeMs.nScenes.toLocaleString()})`,
      formatSuccess(summary.successRate),
      formatDistribution(summary.planningTimeMs),
      formatDistribution(summary.rawPathExcessPct),
      formatDistribution(summary.smoothedPathExcessPct),
      formatDistribution(summary.minimumClearanceM, 2),
    ];
    values.forEach((value) => {
      const cell = document.createElement("td");
      cell.textContent = value;
      row.append(cell);
    });
    body.append(row);
  });
}

async function start(): Promise<void> {
  const bundle = await loadBenchmarkBundle();
  const loadState = element<HTMLParagraphElement>("#load-state");
  const plannerLabels = new Map(bundle.planners.map((planner) => [planner.id, planner.label]));
  const resolutionSeries: ChartSeries[] = (["astar-3d", "lazy-theta-star"] as const).map(
    (plannerId) => ({
      plannerId,
      label: plannerLabels.get(plannerId) ?? plannerId,
      points: bundle.sensitivity.resolution
        .filter((point) => point.plannerId === plannerId)
        .map((point) => ({ x: point.voxelResolutionM, metric: point.rawPathExcessPct })),
    }),
  );
  mountSensitivityChart(element("#resolution-chart"), {
    title: "Voxel-resolution sensitivity",
    description:
      "Raw-path excess for graph planners as voxel resolution changes. Null estimates are omitted rather than drawn at zero.",
    xLabel: "Voxel resolution (m)",
    xScale: "linear",
    formatX: (value) => `${axisNumber(value)} m`,
    series: resolutionSeries,
  });
  mountSensitivityChart(element("#rrt-budget-chart"), {
    title: "RRT* sample-budget sensitivity",
    description:
      "Raw-path excess for RRT* as the fixed sample-attempt budget changes. Null estimates are omitted rather than drawn at zero.",
    xLabel: "Sample-attempt budget (log scale)",
    xScale: "log",
    formatX: compactInteger,
    series: [
      {
        plannerId: "rrt-star",
        label: plannerLabels.get("rrt-star") ?? "RRT*",
        points: bundle.sensitivity.rrtBudget.map((point) => ({
          x: point.sampleBudget,
          metric: point.rawPathExcessPct,
        })),
      },
    ],
  });
  renderSummaryTable(bundle);
  element("#benchmark-meta").textContent =
    `${bundle.dataset.label} (${bundle.dataset.split}); ${bundle.dataset.acceptedScenes} accepted ` +
    `of ${bundle.dataset.attemptedScenes} attempted scenes, with ${bundle.dataset.rejectedScenes} ` +
    `rejections retained in the manifest. Protocol ${bundle.protocol.id}; ` +
    `${bundle.protocol.bootstrap.resamples.toLocaleString()} scene-clustered bootstrap resamples.`;
  const provenance = element("#provenance");
  const sourceLink = document.createElement("a");
  sourceLink.href = `https://github.com/Revincxt/uav-3d-planner/commit/${bundle.sourceCommit}`;
  sourceLink.textContent = bundle.sourceCommit;
  provenance.replaceChildren(
    document.createTextNode(
      `Generated ${new Date(bundle.generatedAt).toLocaleString("en-GB", { timeZone: "UTC" })} UTC from commit `,
    ),
    sourceLink,
    document.createTextNode(
      `. Timing uses ${bundle.protocol.timing.repetitionsPerCell} isolated-process repetitions ` +
        `per cell. Confidence intervals are descriptive and cluster by semantic problem ` +
        `fingerprint; path-quality metrics exclude failed runs while success rates retain them.`,
    ),
  );
  loadState.hidden = true;
}

start().catch((error: unknown) => {
  const state = element<HTMLParagraphElement>("#load-state");
  state.hidden = false;
  state.classList.add("is-error");
  state.textContent =
    error instanceof Error ? error.message : "The recorded benchmark dataset could not load.";
  console.error(error);
});
