import { createRef } from 'react';

import Alerts from '../components/alerts';
import Settings from '../components/settings';
import Splash from '../components/splash';
import Modal from '../components/modal';

const splashRef = createRef<Splash>();
const alertsRef = createRef<Alerts>();
const settingsRef = createRef<Settings>();
const modalRef = createRef<Modal>();

export { splashRef, alertsRef, settingsRef, modalRef };
