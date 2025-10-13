import styles from './sieve.module.css';

import { ros } from '../common/ros';
import { faToggleOn, faToggleOff, faMinus, faPlus } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { useEffect, useRef, useState } from 'react';
import { Topic } from 'roslib';

import Button from '../components/button';
import Input from '../components/input';

let shaftTopic: Topic<unknown> | null = null;
let vibratorsTopic: Topic<unknown> | null = null;

window.addEventListener('ros-connect', () => {
  shaftTopic = new Topic({
    ros: ros,
    name: '/shaft',
    messageType: 'std_msgs/Int32'
  });

  vibratorsTopic = new Topic({
    ros: ros,
    name: '/vibrators',
    messageType: 'std_msgs/Bool'
  });
});

const clamp = (v: number, min = -100, max = 100) => Math.max(min, Math.min(max, v));

export default function Sieve() {
  const inputRef = useRef<Input>(null);

  const [vibratorsEnabled, setVibratorsEnabled] = useState(false);
  const [editing, setEditing] = useState(false);
  const [speed, setSpeed] = useState(0);

  const publishShaftSpeed = () => {
    if (!shaftTopic) return;
    shaftTopic.publish({ data: speed });
  };

  const publishVibratorsState = () => {
    if (!vibratorsTopic) return;
    vibratorsTopic.publish({ data: vibratorsEnabled });
  };

  useEffect(() => {
    inputRef.current?.setValue(speed.toString());
  }, [speed]);

  const handleChange = (text: string) => {
    setEditing(true);
    if (text === '' || text === '-' || text === '+') return;
    let n = parseInt(text, 10);
    if (isNaN(n)) return;
    const clamped = clamp(n);
    if (clamped.toString() !== text) {
      inputRef.current?.setValue(clamped.toString());
    }
    setSpeed(clamped);
  };

  const adjust = (delta: number) => {
    const next = clamp(speed + delta);
    setSpeed(next);
    inputRef.current?.setValue(next.toString());
    setEditing(false);
    publishShaftSpeed();
  };

  return (
    <div className={styles['sieve']}>
      <div className={styles['sieve-rows']}>
        <div className={styles['sieve-row']}>
          <h3 className={styles['sieve-header']}>Main shaft</h3>
        </div>
        <div className={styles['sieve-controls-row']}>
          <Button onClick={() => adjust(-1)} className={styles['red-bg']}>
            <FontAwesomeIcon icon={faMinus} />
          </Button>
          <Input
            type={'number'}
            className={styles['input'] + (editing ? ' editing' : '')}
            defaultValue={speed.toString()}
            ref={inputRef}
            onChange={handleChange}
            onSubmit={publishShaftSpeed}
            onFocus={() => setEditing(true)}
            onBlur={publishShaftSpeed}
          />
          <Button onClick={() => adjust(1)} className={styles['green-bg']}>
            <FontAwesomeIcon icon={faPlus} />
          </Button>
        </div>
        <div className={styles['sieve-row']}>
          <h3 className={styles['sieve-header']}>Vibrators</h3>
        </div>
        <div className={styles['sieve-row']}>
          <Button
            className={`${styles['sieve-button']} ${vibratorsEnabled ? styles['green-bg'] : ''}`}
            tooltip='Enable vibrators'
            onClick={() => {
              setVibratorsEnabled(true);
              publishVibratorsState();
            }}
          >
            <FontAwesomeIcon icon={faToggleOn} />
            &nbsp;&nbsp;Enable
          </Button>
          <Button
            className={`${styles['sieve-button']} ${!vibratorsEnabled ? styles['red-bg'] : ''}`}
            tooltip='Disable vibrators'
            onClick={() => {
              setVibratorsEnabled(false);
              publishVibratorsState();
            }}
          >
            <FontAwesomeIcon icon={faToggleOff} />
            &nbsp;&nbsp;Disable
          </Button>
        </div>
      </div>
    </div>
  );
}
