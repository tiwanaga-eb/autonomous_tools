import MapCanvas from './components/MapCanvas';
import ParameterPanel from './components/ParameterPanel';
import PlannerStatusPanel from './components/PlannerStatusPanel';

export default function App() {
  return (
    <div className="app">
      <ParameterPanel />
      <MapCanvas />
      <PlannerStatusPanel />
    </div>
  );
}
