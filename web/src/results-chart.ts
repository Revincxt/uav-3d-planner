import { uiSymbol, type UISymbol } from "./ui-symbol";
export type StudyKind = "static" | "dynamic" | "predictive";
export type SummaryMetric = "routeM" | "planningTimeMs" | "arrivalTimeS" | "waitTimeS" | "work";
export type ComparisonScale = "linear" | "log1p";
export interface ComparisonPlanner {
  id: string;
  label: string;
  workUnit: string;
  summary: Record<SummaryMetric, number | null>;
}
export interface ComparisonDatum {
  id: string; label: string; shortLabel: string; code: string; color: string;
  value: number | null; unit: string;
}
export interface ComparisonMetric {
  key: SummaryMetric; label: string; shortLabel: string; digits: number;
  entries: ComparisonDatum[];
}
const plannerStyles: Record<string, { color: string; shortLabel: string; code: string }> = {
  "astar-3d": { color: "#68c8ed", shortLabel: "3D A*", code: "A*" },
  "repeated-astar-3d": { color: "#68c8ed", shortLabel: "3D A*", code: "A*" },
  "lazy-theta-star": { color: "#6cdab5", shortLabel: "Lazy Theta*", code: "Θ*" },
  "repeated-lazy-theta-star": { color: "#6cdab5", shortLabel: "Lazy Theta*", code: "Θ*" },
  "rrt-star": { color: "#efba78", shortLabel: "RRT*", code: "RRT*" },
  "dstar-lite-3d": { color: "#b7a1ef", shortLabel: "D* Lite", code: "D*" },
  "dstar-lite-reset-3d": { color: "#b7a1ef", shortLabel: "D* Lite · reset", code: "D*R" },
  "dstar-lite-reuse-3d": { color: "#6cdab5", shortLabel: "D* Lite · reuse", code: "D*U" },
  "space-time-astar-4d": { color: "#efba78", shortLabel: "4D A*", code: "4D" },
};
export const plannerStyle = (id: string, label: string) => plannerStyles[id] ?? { color: "#a6bbc9", shortLabel: label, code: label };
const workUnits: Record<string, string> = {
  "expanded-nodes": "nodes", "queue-pops": "queue pops", "samples": "samples", "expanded-spacetime-states": "states",
};
export function metricComparisons(kind: StudyKind, planners: readonly ComparisonPlanner[]): ComparisonMetric[] {
  const keys: SummaryMetric[] = kind === "static" ? ["routeM", "planningTimeMs", "work"]
    : kind === "dynamic" ? ["routeM", "arrivalTimeS", "work"] : ["routeM", "arrivalTimeS", "waitTimeS"];
  const labels: Record<SummaryMetric, [string, string, string, number]> = {
    routeM: ["Route length", "Path", "km", 2], planningTimeMs: ["Planning time", "Plan", "ms", 1],
    arrivalTimeS: ["Arrival time", "Arrival", "s", 1], waitTimeS: ["Wait time", "Wait", "s", 1], work: ["Search work", "Work", "", 1],
  };
  return keys.map(key => {
    const [label, shortLabel, unit, digits] = labels[key];
    return { key, label, shortLabel, digits, entries: planners.map(planner => {
      const raw = planner.summary[key];
      return { id: planner.id, label: planner.label, ...plannerStyle(planner.id, planner.label),
        value: raw !== null && Number.isFinite(raw) && raw >= 0 ? (key === "routeM" ? raw / 1000 : raw) : null,
        unit: key === "work" ? workUnits[planner.workUnit] ?? planner.workUnit : unit };
    }) };
  });
}

/** Different work units never share a numerical scale or a ranking. */
export function comparisonGroups(metric: ComparisonMetric): Array<{ unit: string; maximum: number; scale: ComparisonScale; entries: ComparisonDatum[] }> {
  const groups = new Map<string, ComparisonDatum[]>();
  metric.entries.forEach(entry => { const group = groups.get(entry.unit) ?? []; group.push(entry); groups.set(entry.unit, group); });
  return [...groups].map(([unit, entries]) => {
    const highest = Math.max(0, ...entries.map(entry => entry.value ?? 0));
    const positive = entries.flatMap(entry => entry.value !== null && entry.value > 0 ? [entry.value] : []);
    const scale: ComparisonScale = metric.key === "planningTimeMs" && positive.length > 1 && highest / Math.min(...positive) >= 100 ? "log1p" : "linear";
    if (highest === 0) return { unit, entries, maximum: 0, scale };
    const magnitude = 10 ** Math.floor(Math.log10(highest));
    const ratio = highest / magnitude;
    return { unit, entries, scale, maximum: magnitude * (ratio <= 1 ? 1 : ratio <= 2 ? 2 : ratio <= 5 ? 5 : 10) };
  });
}
export function barFraction(value: number | null, maximum: number, scale: ComparisonScale = "linear"): number {
  if (value === null || !Number.isFinite(value) || value < 0 || !Number.isFinite(maximum) || maximum <= 0) return 0;
  return Math.min(1, scale === "log1p" ? Math.log1p(value) / Math.log1p(maximum) : value / maximum);
}
/** The logarithmic axis preserves a true zero; ticks use the same transform as bars. */
export function comparisonTicks(maximum: number, scale: ComparisonScale): Array<{ value: number; fraction: number }> {
  const values = maximum <= 0 ? [0] : scale === "log1p"
    ? [0, ...[10, 100, 1000, 10000, 100000, 1000000].filter(value => value < maximum), maximum] : [0, maximum / 2, maximum];
  return values.map(value => ({ value, fraction: barFraction(value, maximum, scale) }));
}
export function metricNumber(value: number | null, digits: number, compact = false): string {
  const abbreviated = compact && value !== null && Math.abs(value) >= 1000;
  return value === null || !Number.isFinite(value) ? "—" : new Intl.NumberFormat("en", { notation: abbreviated ? "compact" : "standard", maximumFractionDigits: abbreviated ? 0 : digits }).format(value);
}
const node = <K extends keyof HTMLElementTagNameMap>(tag: K, className: string, text?: string): HTMLElementTagNameMap[K] => {
  const element = document.createElement(tag); element.className = className;
  if (text !== undefined) element.textContent = text;
  return element;
};
export function renderMetricChart(metric: ComparisonMetric): HTMLElement {
  const figure = node("figure", "comparison-chart"); figure.dataset.metric = metric.key;
  figure.setAttribute("aria-label", `${metric.label}: successful mission medians by algorithm`);
  const groups = comparisonGroups(metric), caption = node("figcaption", "comparison-caption");
  const title = node("span", "comparison-title"); title.title = metric.label;
  title.append(node("span", "comparison-full-title", metric.label), node("span", "comparison-short-title", metric.shortLabel));
  const icons: Record<SummaryMetric, UISymbol> = { routeM: "route", planningTimeMs: "clock", arrivalTimeS: "clock", waitTimeS: "wait", work: "search" };
  title.prepend(uiSymbol(icons[metric.key])); caption.append(title);
  caption.append(node("span", "comparison-unit", groups.length === 1 ? `${groups[0]!.unit}${groups[0]!.scale === "log1p" ? " · log" : ""}` : ""));
  figure.append(caption);
  const body = node("div", "comparison-series"); body.dataset.groups = String(groups.length);
  groups.forEach(group => {
    const section = node("div", "comparison-unit-group");
    section.dataset.unit = group.unit; section.dataset.maximum = String(group.maximum); section.dataset.scale = group.scale;
    if (groups.length > 1) {
      const unit = node("p", "comparison-scale"); unit.title = group.unit; unit.setAttribute("aria-label", group.unit);
      unit.append(node("span", "comparison-full-unit", group.unit), node("span", "comparison-short-unit", group.unit === "queue pops" ? "pops" : group.unit));
      section.append(unit);
    }
    const plot = node("div", "comparison-plot"), axis = node("div", "comparison-y-axis");
    axis.setAttribute("aria-hidden", "true");
    const ticks = comparisonTicks(group.maximum, group.scale);
    for (const { value, fraction } of ticks) {
      const tick = node("span", "comparison-tick", metricNumber(value, metric.digits, true));
      tick.style.bottom = `${fraction * 100}%`; if (fraction > 0 && fraction < 1) tick.classList.add("comparison-middle-tick");
      if (group.scale === "log1p" && fraction > 0 && fraction < 1 && value !== 1000) tick.classList.add("comparison-extra-tick");
      axis.append(tick);
    }
    const columns = node("div", "comparison-columns"); columns.style.setProperty("--column-count", String(group.entries.length));
    group.entries.forEach(entry => {
      const column = node("div", "comparison-column"); column.dataset.planner = entry.id;
      column.dataset.value = entry.value === null ? "" : String(entry.value); column.dataset.unit = entry.unit;
      column.style.setProperty("--planner-color", entry.color);
      column.style.setProperty("--bar-height", `${barFraction(entry.value, group.maximum, group.scale) * 100}%`);
      column.tabIndex = 0;
      column.title = `${entry.label}: ${metricNumber(entry.value, metric.digits)} ${entry.unit}`;
      column.setAttribute("role", "img"); column.setAttribute("aria-label", column.title);
      const name = node("span", "comparison-name");
      name.title = entry.label;
      const tinyCode = entry.code.replace("RRT*", "R*").replace("D*R", "DR").replace("D*U", "DU");
      name.append(node("span", "comparison-full-name", entry.shortLabel), node("span", "comparison-short-name", entry.code), node("span", "comparison-tiny-name", tinyCode));
      const value = node("output", "comparison-value", metricNumber(entry.value, metric.digits, true));
      value.title = `${metricNumber(entry.value, metric.digits)} ${entry.unit}`;
      value.setAttribute("aria-label", `${entry.label}: ${value.title}`);
      const area = node("div", "comparison-column-plot"), bar = node("span", "comparison-bar");
      for (const { fraction } of ticks.filter(tick => tick.fraction > 0)) {
        const line = node("span", "comparison-gridline"); line.style.bottom = `${fraction * 100}%`; line.setAttribute("aria-hidden", "true"); area.append(line);
      }
      bar.setAttribute("aria-hidden", "true"); area.append(bar, value); column.append(area, name); columns.append(column);
    });
    plot.append(axis, columns); section.append(plot);
    body.append(section);
  });
  figure.append(body);
  if (groups.length > 1) figure.title = `${metric.label} · Independent scales for ${groups.map(group => group.unit).join(" / ")}`;
  else if (groups[0]?.scale === "log1p") figure.title = `${metric.label} · log(1 + milliseconds), zero baseline`;
  return figure;
}
