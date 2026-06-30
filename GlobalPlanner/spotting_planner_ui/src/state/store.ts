import { create } from 'zustand';
import type { PlannerParams, Scenario, PlanResult } from '../types';
import { defaultScenario, defaultParams } from '../scenario/defaultScenario';
import { plan } from '../planner/plannerOrchestrator';

type StoreState = {
  scenario: Scenario;
  params: PlannerParams;
  planResult: PlanResult | null;
  planning: boolean;

  setParam: <K extends keyof PlannerParams>(key: K, value: PlannerParams[K]) => void;
  setManualCuspXY: (x: number, y: number) => void;
  setManualCuspHeading: (heading: number) => void;
  resetParams: () => void;
  runPlanner: () => void;
};

export const useStore = create<StoreState>((set, get) => ({
  scenario: defaultScenario,
  params: defaultParams,
  planResult: null,
  planning: false,

  setParam: (key, value) => {
    set((s) => ({ params: { ...s.params, [key]: value } }));
  },
  setManualCuspXY: (x, y) => {
    set((s) => ({ params: { ...s.params, manualCusp: { ...s.params.manualCusp, x, y } } }));
  },
  setManualCuspHeading: (heading) => {
    set((s) => ({ params: { ...s.params, manualCusp: { ...s.params.manualCusp, heading } } }));
  },
  resetParams: () => set({ params: defaultParams }),
  runPlanner: () => {
    set({ planning: true });
    // Run synchronously in the prototype; for production this would move to a Web Worker.
    const { scenario, params } = get();
    const result = plan(scenario, params);
    set({ planResult: result, planning: false });
  },
}));
