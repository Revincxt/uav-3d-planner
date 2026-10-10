export type UISymbol = "benchmark" | "static" | "dynamic" | "predictive" | "route" | "clock" | "search" | "wait" | "close" | "follow";
const paths: Record<UISymbol, string[]> = {
  benchmark: ["M4 18V12h4v6m3 0V7h4v11m3 0V3h4v15", "M2 21h22"],
  static: ["m12 3 8 4.5v9L12 21l-8-4.5v-9L12 3Z", "m4 7.5 8 4.5 8-4.5M12 12v9"],
  dynamic: ["M3 12h4l3-7 4 14 3-7h4", "M3 5h3M18 19h3"],
  predictive: ["M12 4a8 8 0 1 0 8 8M12 8v4l3 2", "M17 3h4v4m0-4-6 6"],
  route: ["M5 5h7a4 4 0 0 1 0 8H9a3 3 0 0 0 0 6h10", "M3 3h4v4H3zM17 17h4v4h-4z"],
  clock: ["M12 3a9 9 0 1 0 0 18 9 9 0 0 0 0-18Z", "M12 7v5l4 2"],
  search: ["M8 8h8v8H8z", "M9 3v3m6-3v3M9 18v3m6-3v3M3 9h3m-3 6h3m12-6h3m-3 6h3"],
  wait: ["M7 3h10M7 21h10M8 3v4l8 10v4M16 3v4L8 17v4"],
  close: ["m6 6 12 12M18 6 6 18"],
  follow: ["M8 3H3v5m13-5h5v5M3 16v5h5m13-5v5h-5", "m9 15 2-4 4-2-2 4-4 2Z"],
};
/** Repo-native icons share one crisp stroke language. */
export function uiSymbol(name: UISymbol): SVGSVGElement {
  const svg = document.createElementNS("http://www.w3.org/2000/svg", "svg");
  svg.setAttribute("viewBox", "0 0 24 24"); svg.setAttribute("class", "ui-symbol");
  svg.setAttribute("fill", "none"); svg.setAttribute("stroke", "currentColor"); svg.setAttribute("stroke-width", "1.5");
  svg.setAttribute("stroke-linecap", "round"); svg.setAttribute("stroke-linejoin", "round");
  svg.setAttribute("aria-hidden", "true"); svg.setAttribute("focusable", "false");
  for (const d of paths[name]) { const path = document.createElementNS(svg.namespaceURI, "path"); path.setAttribute("d", d); svg.append(path); }
  return svg;
}
