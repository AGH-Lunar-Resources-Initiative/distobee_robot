import styles from './pipes.module.css';

import pipeUrl from '!!url-loader!../media/distobee-pipe.svg';
import { ros } from '../common/ros';
import { faArrowLeft, faArrowRight } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { useEffect, useState } from 'react';
import { Topic } from 'roslib';

import Input from '../components/input';
import Label from '../components/label';
import { PipeStates } from '../common/ros-interfaces';

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
    return () => window.removeEventListener('pipe-states', setPipeStates);
  }, []);

  const maskStyle = {
    WebkitMask: `url(${pipeUrl}) center / contain no-repeat`,
    mask: `url(${pipeUrl}) center / contain no-repeat`
  } as const;

  return (
    <div className={styles['pipes']}>
      <div className={styles['pipes-stage']}>
        <div className={`${styles['pipes-group']} ${styles['left']} ${leftAlert ? styles['pipe-state-alert'] : ''}`}>
          <div className={styles['pipes-row']}>
            <Label color='currentColor'>
              <FontAwesomeIcon icon={faArrowLeft} style={{ color: 'white'}} />
            </Label>
            <Input className={styles['pipe-state-input']} defaultValue='Left' disabled />
          </div>
          <div className={styles['pipe-state']}>
            <div className={styles['pipe-state-icon']} style={maskStyle} />
          </div>
        </div>

        <div className={`${styles['pipes-group']} ${styles['right']} ${rightAlert ? styles['pipe-state-alert'] : ''}`}>
          <div className={styles['pipes-row']}>
            <Input className={styles['pipe-state-input']} defaultValue='Right' disabled />
            <Label color='currentColor'>
              <FontAwesomeIcon icon={faArrowRight} style={{ color: 'white'}} />
            </Label>
          </div>
          <div className={styles['pipe-state']}>
            <div className={styles['pipe-state-icon']} style={maskStyle} />
          </div>
        </div>
      </div>
    </div>
  );
}
