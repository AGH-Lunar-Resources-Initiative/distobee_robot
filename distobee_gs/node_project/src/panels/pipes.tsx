import styles from './pipes.module.css';

import { useEffect, useState } from 'react';
import { ros } from '../common/ros';
import { Topic } from 'roslib';
import pipeUrl from '!!url-loader!../media/distobee-pipe.svg';

export type PipeStates = {
  left?: boolean;
  right?: boolean;
};

let lastPipeStates: PipeStates | null = null;

window.addEventListener('ros-connect', () => {
  const topic = new Topic({
    ros,
    name: '/pipe_states',
    messageType: 'distobee_interfaces/PipeStates'
  });

  topic.subscribe((msg: PipeStates) => {
    lastPipeStates = msg;
    window.dispatchEvent(new Event('pipe-states'));
  });
});

export default function Pipes() {
  const [leftAlert, setLeftAlert] = useState(false);
  const [rightAlert, setRightAlert] = useState(false);

  const setPipeStates = () => {
    setLeftAlert(Boolean(lastPipeStates?.left));
    setRightAlert(Boolean(lastPipeStates?.right));
  };

  useEffect(() => {
    window.addEventListener('pipe-states', setPipeStates);
    return () => {
      window.removeEventListener('pipe-states', setPipeStates)
    };
  }, []);

  const pipeMaskStyle = {
    WebkitMaskImage: `url(${pipeUrl})`,
    maskImage: `url(${pipeUrl})`,
  } as const;

  return (
    <div className={styles.pipes}>
      <div className={styles.wrapper}>
        <div className={`${styles['pipe-state']} ${leftAlert ? styles['pipe-state-alert'] : ''}`}>
          <div className={styles.icon} style={pipeMaskStyle} />
        </div>
        <div className={`${styles['pipe-state']} ${rightAlert ? styles['pipe-state-alert'] : ''}`}>
          <div className={styles.icon} style={pipeMaskStyle} />
        </div>
      </div>
    </div>
  );
}
