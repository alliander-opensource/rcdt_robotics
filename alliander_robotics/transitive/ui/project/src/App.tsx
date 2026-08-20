// # SPDX-FileCopyrightText: Alliander N. V.
//
// # SPDX-License-Identifier: Apache-2.0

import './App.css';
import { Teleoperation } from './Teleoperation';

function App() {
  return (
    <>
      <section id="center">
        <div>
          <h1>Alliander Robotics Dashboard</h1>
          <Teleoperation />
        </div>
      </section>

    </>
  )
}

export default App
