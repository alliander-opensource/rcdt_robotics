// # SPDX-FileCopyrightText: Alliander N. V.
//
// # SPDX-License-Identifier: Apache-2.0

import { CapabilityContext, CapabilityContextProvider } from '@transitive-sdk/utils-web';
import { useContext, useEffect, useState } from 'react';
import './card.css';
import { generateJWT } from './jwt';

export interface Subscription {
  topic: string;
  fields: string[];
  callback: (data: any) => void;
}

const Subscriber = ({ subscriptions }: { subscriptions: Subscription[] }) => {
  // import the API exposed by the ros-tool capability
  const { isReady, subscribe, unsubscribe, deviceData } = useContext(CapabilityContext)
  const messages = deviceData?.ros?.[2].messages

  useEffect(() => {
    // Subscribe to the topics:
    if (isReady?.()) {
      subscriptions.forEach(sub => subscribe(2, sub.topic));
    }

    // Unsubscribe when React component unmounts:
    return () => {
      subscriptions.forEach(sub => unsubscribe?.(2, sub.topic));
    }
  }, [isReady, subscribe])

  // Create useEffect hooks for each subscription:
  for (const sub of subscriptions) {

    // Define the variables based on the fields of the subscription:
    let variables = []
    for (const field of sub.fields) {
      let attributes = sub.topic.concat(field).split('/').filter(attr => attr !== '');
      let variable = messages;
      for (const attr of attributes) {
        variable = variable?.[attr];
      }
      variables.push(variable);
    }

    // The subscription callback will be called whenever one of the relevant variables changes:
    useEffect(() => {
      sub.callback(variables);
    }, variables);
  }

  return (
    <pre>
      {messages ? JSON.stringify(messages, null, 2) : 'No messages received.'}
    </pre>
  );
}

export function RosTool({ device, subscriptions }: { device: string, subscriptions: Subscription[] }) {
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
      <Subscriber subscriptions={subscriptions} />
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