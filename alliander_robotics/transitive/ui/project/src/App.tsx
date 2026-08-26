// # SPDX-FileCopyrightText: Alliander N. V.
//
// # SPDX-License-Identifier: Apache-2.0

import { useState } from 'react';
import './App.css';
import { Teleoperation } from './Teleoperation';

function App() {
  const [device, setDevice] = useState<'simulation' | 'lynx' | 'panther'>('lynx');

  const deviceSelector = <div>
    <label>
      <input
        type="radio"
        checked={device === 'simulation'}
        onChange={() => setDevice("simulation")} />
      simulation
    </label>
    <label>
      <input
        type="radio"
        checked={device === 'lynx'}
        onChange={() => setDevice("lynx")} />
      lynx
    </label>
    <label>
      <input
        type="radio"
        checked={device === 'panther'}
        onChange={() => setDevice("panther")} />
      panther
    </label>
  </div>

  return (
    <>
      <section id="center">
        <div>
          <h1>Alliander Robotics Dashboard</h1>
          {deviceSelector}
          <Teleoperation device={device}/>
        </div>
      </section>

    </>
  )
}

export default App
