import styles from './wheels.module.css';

import distobeeBody from '!!url-loader!../media/distobee-body.svg';
import distobeeLeftWheelOutline from '!!url-loader!../media/distobee-wheel-outline.svg';
import distobeeLeftWheel from '!!url-loader!../media/distobee-wheel.svg';
import { ros } from '../common/ros';
import { WheelStates, WheelTelemetry } from '../common/ros-interfaces';
import { MutableRefObject, useCallback, useEffect, useLayoutEffect, useRef, useState } from 'react';
import { Topic, Service } from 'roslib';
import { AxisStateRequest, AxisStateResponse, EmptyRequest, EmptyResponse } from '../common/ros-interfaces';
import { faToggleOn, faToggleOff, faExclamationTriangle } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import Button from '../components/button';

let lastWheelStates: WheelStates | null = null;
let lastWheelStatesReturn: WheelStates | null = null;

let lastWheelTelemetryFl: WheelTelemetry | null = null;
let lastWheelTelemetryFr: WheelTelemetry | null = null;
let lastWheelTelemetryBl: WheelTelemetry | null = null;
let lastWheelTelemetryBr: WheelTelemetry | null = null;

let odriveSwitchSrv: Service<AxisStateRequest, AxisStateResponse> = null;
let odriveClearErrorsSrv: Service<EmptyRequest, EmptyResponse> = null;
let wheelsTopic: Topic<unknown> | null = null;

window.addEventListener('ros-connect', () => {
  const targetTopic = new Topic({
    ros: ros,
    name: '/wheel_states/target',
    messageType: 'distobee_interfaces/WheelStates'
  });

  targetTopic.subscribe((msg: WheelStates) => {
    lastWheelStates = msg;
    window.dispatchEvent(new Event('wheel-states-target'));
  });

  const currentTopic = new Topic({
    ros: ros,
    name: '/wheel_states/current',
    messageType: 'distobee_interfaces/WheelStates'
  });

  currentTopic.subscribe((msg: WheelStates) => {
    lastWheelStatesReturn = msg;
    window.dispatchEvent(new Event('wheel-states-current'));
  });

  const telemetryFl = new Topic({
    ros,
    name: '/wheel_states/telemetry/fl',
    messageType: 'distobee_interfaces/WheelTelemetry'
  });

  telemetryFl.subscribe((msg: WheelTelemetry) => {
    lastWheelTelemetryFl = msg;
    console.log(msg);
    window.dispatchEvent(new Event('wheel-telemetry-fl'));
  });

  const telemetryFr = new Topic({
    ros,
    name: '/wheel_states/telemetry/fr',
    messageType: 'distobee_interfaces/WheelTelemetry'
  });

  telemetryFr.subscribe((msg: WheelTelemetry) => {
    lastWheelTelemetryFr = msg;
    window.dispatchEvent(new Event('wheel-telemetry-fr'));
  });

  const telemetryBl = new Topic({
    ros,
    name: '/wheel_states/telemetry/bl',
    messageType: 'distobee_interfaces/WheelTelemetry'
  });

  telemetryBl.subscribe((msg: WheelTelemetry) => {
    lastWheelTelemetryBl = msg;
    window.dispatchEvent(new Event('wheel-telemetry-bl'));
  });

  const telemetryBr = new Topic({
    ros,
    name: '/wheel_states/telemetry/br',
    messageType: 'distobee_interfaces/WheelTelemetry'
  });

  telemetryBr.subscribe((msg: WheelTelemetry) => {
    lastWheelTelemetryBr = msg;
    window.dispatchEvent(new Event('wheel-telemetry-br'));
  });

  // Topic for enabling/disabling wheels from the GUI (kept for backward compatibility if needed)
  // publishes std_msgs/Bool with current enabled/disabled state
  wheelsTopic = new Topic<unknown>({
    ros,
    name: '/wheels/enable',
    messageType: 'std_msgs/Bool'
  });

  // Service to request ODrive axis state change via the odrive_state_switcher node
  odriveSwitchSrv = new Service<AxisStateRequest, AxisStateResponse>({
    ros,
    name: '/odrive_switch_state',
    serviceType: 'odrive_can/srv/AxisState'
  });

  // Service to clear ODrive errors via the odrive_state_switcher node
  odriveClearErrorsSrv = new Service<EmptyRequest, EmptyResponse>({
    ros,
    name: '/odrive_clear_errors',
    serviceType: 'std_srvs/srv/Empty'
  });
});

type ArrowProps = {
  velocity: number;
  angle: number;
  thickness: number;
};

function Arrow({ velocity, angle, thickness }: ArrowProps) {
  const opacity = Math.pow(Math.min(1, Math.abs(velocity)), 0.25);
  const scale = Math.sign(velocity) * Math.pow(Math.min(1, Math.max(0, Math.abs(velocity))), 0.5);

  return (
    <div
      className={styles['arrow']}
      style={{
        transform: `rotate(${angle - 90}deg) scale(${scale})`,
        opacity,
        borderWidth: thickness * 0.5,
        left: `calc(48% - ${thickness * 0.5}px)`,
        top: `calc(50% - ${thickness * 0.5}px)`,
        transformOrigin: `${thickness * 0.5}px ${thickness * 0.5}px`
      }}
    >
      <div />
      <div
        className={styles['arrow-head']}
        style={{
          borderTopWidth: thickness,
          borderRightWidth: thickness,
          marginRight: -thickness,
          width: thickness * 2,
          height: thickness * 2
        }}
      />
    </div>
  );
}

type TelemetryProps = {
  panelId: string;
  height: number;
  type: 'fl' | 'fr' | 'bl' | 'br';
  telemetry: Record<string, any>;
  showTelemetry: boolean;
  velocity: number;
};

function Telemetry({ panelId, height, type, telemetry, showTelemetry, velocity }: TelemetryProps) {
  // Custom component displaying wheel telemetry from distobee_interfaces/WheelTelemetry.
  // Adding new telemetry fields requires updating the interface, its TS type in ros-interfaces.ts,
  // and extending this component to render them.
  const parentOffsetWidth = (document.getElementById(panelId) as HTMLElement)?.offsetWidth ?? 0;

  const axisStateMap: Record<number, string> = {
    0: 'UNDEFINED',
    1: 'IDLE',
    2: 'STARTUP_SEQUENCE',
    3: 'FULL_CALIBRATION_SEQUENCE',
    4: 'MOTOR_CALIBRATION',
    6: 'ENCODER_INDEX_SEARCH',
    7: 'ENCODER_OFFSET_CALIBRATION',
    8: 'CLOSED_LOOP_CONTROL',
    9: 'LOCKIN_SPIN',
    10: 'ENCODER_DIR_FIND',
    11: 'HOMING',
    12: 'ENCODER_HALL_POLARITY_CALIBRATION'
  };

  if (!showTelemetry) {
    return (
      <div className={styles['wheel-telemetry'] + ' ' + styles[type]} style={{ height }}>
        <p className={'danger'}>N/A</p>
      </div>
    );
  }

  return (
    <div className={styles['wheel-telemetry'] + ' ' + styles[type]} style={{ height }}>
      <p>
        <b>
          <span className={styles['wheel-telemetry-data']}>{Number(velocity).toFixed(2)} rad/s</span>
        </b>
      </p>
      <p>
        {parentOffsetWidth > 600 && <>State: </>}
        <span className={styles['wheel-telemetry-data']}>{axisStateMap[telemetry?.state ?? 0]}</span>
      </p>
    </div>
  );
}

type WheelProps = {
  panelId: string;
  type: 'fl' | 'fr' | 'bl' | 'br';
  angle: number;
  velocity: number;
  telemetry: Record<string, any>;
  showTarget: boolean;
  showTelemetry: boolean;
};

function Wheel({ panelId, type, angle, velocity, telemetry, showTarget, showTelemetry }: WheelProps) {
  const ref = useRef<HTMLDivElement | null>(null);
  const thickness = ref.current ? ref.current.clientWidth * 0.3 : 5;
  const [height, setHeight] = useState(0);

  useLayoutEffect(() => {
    if (!ref.current) return;
    const updateSize = () => {
      setHeight(ref.current.offsetHeight);
    };

    const ro = new ResizeObserver(updateSize);
    ro.observe(ref.current);

    updateSize();

    return () => ro.disconnect();
  }, []);

  return (
    <>
      {!showTarget && (
        <Telemetry
          panelId={panelId}
          height={height}
          type={type}
          telemetry={telemetry}
          showTelemetry={showTelemetry}
          velocity={velocity}
        />
      )}
      <div
        ref={ref}
        className={styles['wheel'] + ' ' + styles[type]}
        style={{
          transform: `rotate(${(-angle * 180) / Math.PI + (type === 'fr' || type === 'br' ? 180 : 0)}deg)`
        }}
      >
        <img
          src={showTarget ? distobeeLeftWheelOutline : distobeeLeftWheel}
          className={styles['wheel-image']}
          draggable='false'
          alt=''
        />
        <Arrow velocity={velocity} angle={type === 'fr' || type === 'br' ? 180 : 0} thickness={thickness} />
      </div>
    </>
  );
}

type RoverProps = {
  panelId: string;
  showTarget?: boolean;
};

function Rover({ panelId, showTarget = false }: RoverProps) {
  const [_, setRerenderCount] = useState(0);
  const [showTelemetry, setShowTelemetry] = useState({
    fl: false,
    fr: false,
    bl: false,
    br: false
  });

  const rerender = useCallback(() => {
    setRerenderCount((count) => count + 1);
  }, []);

  const handleWheelTelemetry = useCallback(
    (wheel: 'fl' | 'fr' | 'bl' | 'br') => {
      rerender();
      setShowTelemetry((prev) => ({ ...prev, [wheel]: true }));
    },
    [rerender]
  );

  useEffect(() => {
    const onFlTelemetry = () => handleWheelTelemetry('fl');
    const onFrTelemetry = () => handleWheelTelemetry('fr');
    const onBlTelemetry = () => handleWheelTelemetry('bl');
    const onBrTelemetry = () => handleWheelTelemetry('br');

    window.addEventListener('wheel-states-target', rerender);
    window.addEventListener('wheel-states-current', rerender);
    window.addEventListener('resize', rerender);

    window.addEventListener('wheel-telemetry-fl', onFlTelemetry);
    window.addEventListener('wheel-telemetry-fr', onFrTelemetry);
    window.addEventListener('wheel-telemetry-bl', onBlTelemetry);
    window.addEventListener('wheel-telemetry-br', onBrTelemetry);
    return () => {
      window.removeEventListener('wheel-states-target', rerender);
      window.removeEventListener('wheel-states-current', rerender);
      window.removeEventListener('resize', rerender);

      window.removeEventListener('wheel-telemetry-fl', onFlTelemetry);
      window.removeEventListener('wheel-telemetry-fr', onFrTelemetry);
      window.removeEventListener('wheel-telemetry-bl', onBlTelemetry);
      window.removeEventListener('wheel-telemetry-br', onBrTelemetry);
    };
  }, [rerender]);

  const wheelStates = showTarget ? lastWheelStates : lastWheelStatesReturn;
  const data = mapWheelData(wheelStates);

  return (
    <div className={styles['rover'] + (showTarget ? ' show-target' : '')}>
      <div className={styles['rover-h']}>
        <div className={styles['rover-v']}>
          {!showTarget && <img src={distobeeBody} className={styles['body']} draggable='false' alt='' />}
          <Wheel
            panelId={panelId}
            type='fl'
            angle={data.fl.angle}
            velocity={data.fl.velocity}
            telemetry={lastWheelTelemetryFl}
            showTarget={showTarget}
            showTelemetry={showTelemetry.fl}
          />
          <Wheel
            panelId={panelId}
            type='fr'
            angle={data.fr.angle}
            velocity={data.fr.velocity}
            telemetry={lastWheelTelemetryFr}
            showTarget={showTarget}
            showTelemetry={showTelemetry.fr}
          />
          <Wheel
            panelId={panelId}
            type='bl'
            angle={data.bl.angle}
            velocity={data.bl.velocity}
            telemetry={lastWheelTelemetryBl}
            showTarget={showTarget}
            showTelemetry={showTelemetry.bl}
          />
          <Wheel
            panelId={panelId}
            type='br'
            angle={data.br.angle}
            velocity={data.br.velocity}
            telemetry={lastWheelTelemetryBr}
            showTarget={showTarget}
            showTelemetry={showTelemetry.br}
          />
        </div>
      </div>
  {/* controls moved to parent Wheels component to avoid duplication */}
    </div>
  );
}

export default function Wheels() {
  const id = `wheels-panel-${Math.random().toString(36).substring(2, 4)}`;

  return (
    <div id={id} className={styles['wheels']}>
      <div className={styles['rover-wrapper']}>
        <Rover panelId={id} />
        <Rover panelId={id} showTarget />
      </div>
      <div className={styles['wheels-controls-wrapper']}>
        <div className={styles['wheels-controls']} role="group" aria-label="Wheels controls">
          <Button
            className={`${styles['sieve-button']}`}
            tooltip='Enable wheels (CLOSED_LOOP)'
            onClick={() => {
              const svc = odriveSwitchSrv;
              if (svc) {
                svc.callService({ axis_requested_state: 8 }, (result: any) => {
                  // result handling optional
                  console.log('odrive switch result', result);
                });
              }
              (document.getElementById(id) as HTMLElement)?.setAttribute('data-wheels-enabled', 'true');
            }}
          >
            <FontAwesomeIcon icon={faToggleOn} />
            &nbsp;&nbsp;Enable
          </Button>
          <Button
            className={`${styles['sieve-button']} ${styles['red-bg']}`}
            tooltip='Disable wheels (IDLE)'
            onClick={() => {
              const svc = odriveSwitchSrv;
              if (svc) {
                svc.callService({ axis_requested_state: 1 }, (result: any) => {
                  console.log('odrive switch result', result);
                });
              }
              (document.getElementById(id) as HTMLElement)?.setAttribute('data-wheels-enabled', 'false');
            }}
          >
            <FontAwesomeIcon icon={faToggleOff} />
            &nbsp;&nbsp;Disable
          </Button>
        </div>
        <div className={styles['wheels-controls']}>
          <Button
            className={`${styles['sieve-button']} ${styles['yellow-bg']}`}
            tooltip='Clear ODrive errors'
            onClick={() => {
              const svc = odriveClearErrorsSrv;
              if (svc) {
                svc.callService({}, (result: any) => {
                  console.log('odrive clear errors result', result);
                });
              }
            }}
          >
            <FontAwesomeIcon icon={faExclamationTriangle} />
            &nbsp;&nbsp;Clear Errors
          </Button>
        </div>
      </div>
    </div>
  );
}

function mapWheelData(ws: WheelStates | null) {
  return {
    fl: {
      angle: ws?.front_left_angle ?? 0,
      velocity: ws?.back_left_velocity ?? 0
    },
    fr: {
      angle: ws?.front_right_angle ?? 0,
      velocity: ws?.back_right_velocity ?? 0
    },
    bl: {
      angle: 0,
      velocity: ws?.back_left_velocity ?? 0
    },
    br: {
      angle: 0,
      velocity: ws?.back_right_velocity ?? 0
    }
  } as const;
}
