// # SPDX-FileCopyrightText: Alliander N. V.
//
// # SPDX-License-Identifier: Apache-2.0

import { useState } from 'react';
import './App.css';
import { HealthMonitor } from './capabilities/HealthMonitor';
import { RosTool } from './capabilities/RosTool';
import { Teleoperation } from './capabilities/Teleoperation';
import { Map } from './Map';

type Device = 'simulation' | 'lynx' | 'panther' | 'none';
const DEVICE: Device = 'simulation';

function App() {
  const device_stored = sessionStorage.getItem(DEVICE) as Device | null;
  const [device] = useState<Device>(device_stored ?? DEVICE);
  const [position, setPosition] = useState<[number, number] | null>(null);

  const handleDeviceChange = (value: Device) => {
    sessionStorage.setItem(DEVICE, value);
    window.location.reload();
  };

  const deviceSelector = <div>
    <label>
      <input
        type="radio"
        checked={device === 'simulation'}
        onChange={() => handleDeviceChange("simulation")} />
      simulation
    </label>
    <label>
      <input
        type="radio"
        checked={device === 'lynx'}
        onChange={() => handleDeviceChange("lynx")} />
      lynx
    </label>
    <label>
      <input
        type="radio"
        checked={device === 'panther'}
        onChange={() => handleDeviceChange("panther")} />
      panther
    </label>
    <label>
      <input
        type="radio"
        checked={device === 'none'}
        onChange={() => handleDeviceChange("none")} />
      none
    </label>
  </div>

  const teleoperation = <Teleoperation device={device} />;
  const healthMonitor = <HealthMonitor device={device} />;
  const rosTool = <RosTool device={device} setPosition={setPosition} />;

  const map = <Map position={position} />;

  return (
    <>
      <div className="app">
        <div className="title">
          <h1>Alliander Robotics Dashboard</h1>
          {deviceSelector}
        </div>
        <div className="cards">
          {teleoperation}
          {healthMonitor}
          {rosTool}
        </div>
        <div className="map">
          {map}
        </div>
      </div>
    </>
  );
}

export default App
