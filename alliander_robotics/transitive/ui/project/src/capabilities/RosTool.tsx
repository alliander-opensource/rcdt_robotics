// # SPDX-FileCopyrightText: Alliander N. V.
//
// # SPDX-License-Identifier: Apache-2.0

import { CapabilityContext, CapabilityContextProvider } from '@transitive-sdk/utils-web';
import { useContext, useEffect, useState } from 'react';
import './card.css';
import { generateJWT } from './jwt';

const Subscriber = ({ setPosition }: { setPosition: (value: [number, number] | null) => void }) => {
  // import the API exposed by the ros-tool capability
  const { isReady, subscribe, unsubscribe, deviceData } = useContext(CapabilityContext)

  useEffect(() => {
    if (isReady?.()) {
      subscribe(2, '/ublox/gps/fix');
    }

    // unsubscribe when React component unmounts
    return () => {
      unsubscribe?.(2, '/ublox/gps/fix');
    }
  }, [isReady, subscribe])

  // Update the messages
  const messages = deviceData?.ros?.[2].messages
  const latitude = messages?.ublox?.gps?.fix?.latitude;
  const longitude = messages?.ublox?.gps?.fix?.longitude;
  useEffect(() => {
    setPosition(latitude && longitude ? [latitude, longitude] : null);
  }, [latitude, longitude]);

  return (
    <pre>
      {messages ? JSON.stringify(messages, null, 2) : 'No messages received.'}
    </pre>
  );
}

export function RosTool({ device, setPosition }: { device: string, setPosition: (value: [number, number] | null) => void }) {
  const [jwtToken, setJwtToken] = useState('');
  const [jwtError, setJwtError] = useState<string | null>(null);
  const [capability, setCapability] = useState(<div></div>);

  // Generate JWT token on mount
  useEffect(() => {
    generateJWT(device, '@transitive-robotics/ros-tool').then(({ jwtToken: token, jwtError: error }) => {
      setJwtToken(token);
      setJwtError(error);
    });
  }, [device]);

  // Update the capability when the JWT token changes
  useEffect(() => {
    if (!jwtToken) {
      return;
    }

    const embed = <CapabilityContextProvider jwt={jwtToken}>
      <Subscriber setPosition={setPosition} />
    </CapabilityContextProvider>

    setCapability(<div className="capability">{embed}</div>);
  }, [jwtToken]);


  // Create a widget
  const widget = (
    <>
      {capability}
    </>
  );

  // Render as a card with a title and the widget.
  return (
    <div className="card">
      <div className="header"><b>ROS Tool ({device})</b></div>
      <div className="widget">
        {jwtError ? <p>Error: {jwtError}</p> : widget}
      </div>
    </div>
  );
}