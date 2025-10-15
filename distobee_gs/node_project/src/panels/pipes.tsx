import styles from './pipes.module.css';

import pipeUrl from '!!url-loader!../media/distobee-pipe.svg';
import { ros } from '../common/ros';
import { faArrowLeft, faArrowRight } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { useEffect, useState } from 'react';
import { Topic } from 'roslib';

import Input from '../components/input';
import Label from '../components/label';
import { PipeStates, ControllerStatus, ControlMessage } from '../common/ros-interfaces';

let lastPipeStates: PipeStates | null = null;
let lastTiltController: ControllerStatus | null = null;
let lastTiltControl: ControlMessage | null = null;
let lastPipesController: ControllerStatus | null = null;
let lastPipesControl: ControlMessage | null = null;

window.addEventListener('ros-connect', () => {
  const pipeStatesTopic = new Topic({
    ros,
    name: '/pipe_states',
    messageType: 'distobee_interfaces/PipeStates'
  });
  pipeStatesTopic.subscribe((msg: PipeStates) => {
    lastPipeStates = msg;
    window.dispatchEvent(new Event('pipe-states'));
  });

  const tiltControllerTopic = new Topic({
    ros,
    name: '/odrive_tilt/controller_status',
    messageType: 'odrive_can/ControllerStatus'
  });
  tiltControllerTopic.subscribe((msg: ControllerStatus) => {
    lastTiltController = msg;
    window.dispatchEvent(new Event('tilt-status'));
  });

  const tiltControlTopic = new Topic({
    ros,
    name: '/odrive_tilt/control_message',
    messageType: 'odrive_can/ControlMessage'
  });
  tiltControlTopic.subscribe((msg: ControlMessage) => {
    lastTiltControl = msg;
    window.dispatchEvent(new Event('tilt-status'));
  });

  const pipesControllerTopic = new Topic({
    ros,
    name: '/odrive_pipes/controller_status',
    messageType: 'odrive_can/ControllerStatus'
  });
  pipesControllerTopic.subscribe((msg: ControllerStatus) => {
    lastPipesController = msg;
    window.dispatchEvent(new Event('pipes-status'));
  });

  const pipesControlTopic = new Topic({
    ros,
    name: '/odrive_pipes/control_message',
    messageType: 'odrive_can/ControlMessage'
  });
  pipesControlTopic.subscribe((msg: ControlMessage) => {
    lastPipesControl = msg;
    window.dispatchEvent(new Event('pipes-status'));
  });
});

export default function Pipes() {
  const [leftAlert, setLeftAlert] = useState(false);
  const [rightAlert, setRightAlert] = useState(false);
  const [tiltEst, setPosEstimate] = useState<number | null>(null);
  const [inputTilt, setInputPos] = useState<number | null>(null);
  const [velEst, setVelEstimate] = useState<number | null>(null);
  const [inputVel, setInputVel] = useState<number | null>(null);

  const setPipeStates = () => {
    setLeftAlert(Boolean(lastPipeStates?.left));
    setRightAlert(Boolean(lastPipeStates?.right));
  };

  const setTiltStatus = () => {
    setPosEstimate(lastTiltController?.pos_estimate ?? null);
    setInputPos(lastTiltControl?.input_pos ?? null);
  };

  const setPipesStatus = () => {
    setVelEstimate(lastPipesController?.vel_estimate ?? null);
    setInputVel(lastPipesControl?.input_vel ?? null);
  };

  useEffect(() => {
    window.addEventListener('pipe-states', setPipeStates);
    window.addEventListener('tilt-status', setTiltStatus);
    window.addEventListener('pipes-status', setPipesStatus);
    return () => {
      window.removeEventListener('pipe-states', setPipeStates);
      window.removeEventListener('tilt-status', setTiltStatus);
      window.removeEventListener('pipes-status', setPipesStatus);
    };
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

      <div className={styles['tilt-status']}>
        <div className={styles['tilt-row']}>
          <Label color='var(--blue-background)'>Tilt Est.</Label>
          <Label 
            className={styles['tilt-value']} 
            color='var(--dark-background)'
          >
            {tiltEst !== null ? tiltEst.toFixed(3) : '--'}
          </Label>
          <Label color='var(--red-background)'>Target Tilt</Label>
          <Label 
            className={styles['tilt-value']} 
            color='var(--dark-background)'
          >
            {inputTilt !== null ? inputTilt.toFixed(3) : '--'}
          </Label>
        </div>
        <div className={styles['tilt-row']}>
          <Label color='var(--blue-background)'>Screw Vel Est.</Label>
          <Label 
            className={styles['tilt-value']} 
            color='var(--dark-background)'
          >
            {velEst !== null ? velEst.toFixed(3) : '--'}
          </Label>
          <Label color='var(--red-background)'>Target Vel</Label>
          <Label 
            className={styles['tilt-value']} 
            color='var(--dark-background)'
          >
            {inputVel !== null ? inputVel.toFixed(3) : '--'}
          </Label>
        </div>
      </div>
    </div>
  );
}
