import styles from './sieve.module.css';

import { ros } from '../common/ros';
import {
  faWeightHanging,
  faFlask,
  faBoxOpen,
  faBox,
  faArrowRotateRight,
  faDroplet,
  faTrash,
  faList,
  faBan,
  faMagnet,
  faArrowDown,
  faArrowUp,
  faStop,
  faDiagramProject,
  faRuler,
  faRobot,
  faOilWell,
  faPlay
} from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { useEffect, useState, useRef } from 'react';
import { Topic, Service } from 'roslib';

import Button from '../components/button';
import Dropdown from '../components/dropdown';
import Label from '../components/label';

window.addEventListener('ros-connect', () => {});

type SievePanelProps = {
  props: {};
};

type PHProbeProps = {};

export default function Sieve({ props }: SievePanelProps) {
  return <div className={styles['sieve']}></div>;
}
