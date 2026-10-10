import type { CityMission } from "./city-schema";
import { routeColorCSS } from "./route-overview";

interface LibraryScenario {
  id: string;
  label: string;
  mission?: CityMission;
}
interface WorkspaceOptions {
  scenarios: LibraryScenario[];
  select: HTMLSelectElement;
}

/** All missions stay visible; the legend changes details and follow target only. */
export function mountWorkspace({ scenarios, select }: WorkspaceOptions): void {
  if (!scenarios.length) return;
  const focus = (id: string): void => {
    if (select.value === id) return;
    select.value = id;
    select.dispatchEvent(new Event("change", { bubbles: true }));
  };
  const legend = document.querySelector<HTMLElement>("#legend, .scene-legend, .canvas-legend");
  const legendButtons: HTMLButtonElement[] = [];
  if (legend) {
    legend.replaceChildren();
    legend.classList.add("route-legend");
    const header = document.querySelector(".app-header");
    header?.prepend(legend);
    legend.setAttribute("aria-label", `${scenarios.length} tasks in shared airspace; click to observe or follow a task`);
    legend.title = "All tasks share the same obstacles and clock. Focus changes stops and follow target only; inter-mission avoidance is not jointly optimized.";
    scenarios.forEach((scenario, index) => {
      const button = document.createElement("button");
      button.type = "button";
      button.dataset.route = scenario.id;
      button.style.setProperty("--route-color", routeColorCSS(index));
      const swatch = document.createElement("i");
      swatch.setAttribute("aria-hidden", "true");
      button.append(swatch, document.createTextNode(String(index + 1).padStart(2, "0")));
      button.title = `${scenario.label}${scenario.mission?.challenge ? ` · ${scenario.mission.challenge.title}` : ""}`;
      button.setAttribute("aria-label", `Focus route ${index + 1}: ${scenario.label}`);
      button.addEventListener("click", () => focus(scenario.id));
      legend.append(button);
      legendButtons.push(button);
    });
  }
  const synchronize = (): void => {
    const index = scenarios.findIndex(s => s.id === select.value);
    for (const button of legendButtons) {
      button.setAttribute("aria-pressed", String(button.dataset.route === select.value));
    }
    document.documentElement.style.setProperty("--active-route", routeColorCSS(Math.max(0, index)));
    const inspector = document.querySelector<HTMLElement>(".inspector-card");
    if (inspector) {
      inspector.dataset.route = select.value;
      inspector.setAttribute("aria-label", `Focused mission details: ${scenarios[index]?.label ?? "Study"}`);
      const heading = inspector.querySelector<HTMLElement>(".inspector-heading h2, .inspector-header h2");
      if (heading) {
        heading.textContent = `UAV ${index + 1}`;
        heading.title = scenarios[index]?.label ?? "";
      }
    }
  };
  select.addEventListener("change", synchronize);
  synchronize();
}

export function mountInspector(): void {
  const panel = document.querySelector<HTMLElement>(".inspector-card");
  const toolbar = document.querySelector(".stage-toolbar");
  if (!panel || !toolbar) return;
  panel.id ||= "flight-inspector";
  const toggle = document.createElement("button");
  toggle.type = "button";
  toggle.className = "inspector-toggle";
  toggle.textContent = "Inspector";
  toggle.title = "Open inspector";
  toggle.setAttribute("aria-controls", panel.id);
  toggle.setAttribute("aria-expanded", "false");
  panel.inert = true;
  const close = document.createElement("button");
  close.type = "button";
  close.className = "inspector-close";
  close.textContent = "×";
  close.setAttribute("aria-label", "Close inspector");
  const dismiss = (): void => {
    panel.classList.remove("inspector-open");
    panel.inert = true;
    toggle.setAttribute("aria-expanded", "false");
    toggle.title = "Open inspector";
  };
  close.addEventListener("click", () => { dismiss(); toggle.focus(); });
  panel.append(close);
  toggle.addEventListener("click", () => {
    const open = panel.classList.toggle("inspector-open");
    panel.inert = !open;
    toggle.setAttribute("aria-expanded", String(open));
    toggle.title = open ? "Close inspector" : "Open inspector";
  });
  toolbar.append(toggle);
  document.addEventListener("keydown", (event) => {
    if (event.key === "Escape" && panel.classList.contains("inspector-open")) {
      dismiss();
      toggle.focus();
    }
  });
}

export function mountTabs(): void {
  document.querySelectorAll<HTMLElement>("[data-tab-group]").forEach((group) => {
    const buttons = [...group.querySelectorAll<HTMLButtonElement>("[data-panel]")];
    const activate = (button: HTMLButtonElement): void => {
      buttons.forEach((candidate) => {
        const active = candidate === button;
        candidate.setAttribute("aria-selected", String(active));
        candidate.tabIndex = active ? 0 : -1;
        const panel = document.getElementById(candidate.dataset.panel!);
        if (panel) panel.hidden = !active;
      });
      window.dispatchEvent(new Event("resize"));
    };
    buttons.forEach((button, index) => {
      button.addEventListener("click", () => activate(button));
      button.addEventListener("keydown", (event) => {
        if (!["ArrowLeft", "ArrowRight", "Home", "End"].includes(event.key)) return;
        event.preventDefault();
        const next = event.key === "Home" ? 0 : event.key === "End" ? buttons.length - 1 :
          (index + (event.key === "ArrowRight" ? 1 : -1) + buttons.length) % buttons.length;
        activate(buttons[next]!);
        buttons[next]!.focus();
      });
    });
    const selected = buttons.find(button => button.getAttribute("aria-selected") === "true") ?? buttons[0];
    if (selected) activate(selected);
  });
}
