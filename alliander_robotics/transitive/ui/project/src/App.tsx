// # SPDX-FileCopyrightText: Alliander N. V.
//
// # SPDX-License-Identifier: Apache-2.0

import { useState } from 'react';
import './App.css';
import { HealthMonitor } from './HealthMonitor';
import { Teleoperation } from './Teleoperation';

type Device = 'simulation' | 'lynx' | 'panther' | 'none';
const DEVICE: Device = 'simulation';

function App() {
  const device_stored = sessionStorage.getItem(DEVICE) as Device | null;
  const [device] = useState<Device>(device_stored ?? DEVICE);

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

  return (
    <>
      <div className="app">
        <div className="title">
          <h1>Alliander Robotics Dashboard</h1>
          {deviceSelector}
        </div>
        <div className="cards">
          <Teleoperation device={device} />
          <HealthMonitor device={device} />
        </div>
      </div>
    </>
  )
}

export default App
