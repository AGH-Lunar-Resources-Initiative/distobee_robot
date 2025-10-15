import styles from './feeds.module.css';

import { feedCameras } from '../common/feeds';
import { IconDefinition, faCar, faFlask, faCamera, faCheck, faDisplay } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { useCallback, useEffect, useRef, useState } from 'react';

import Button from '../components/button';
import Input from '../components/input';
import Label from '../components/label';

type IntegerSelectorProps = {
  labelColor: string;
  labelIcon: IconDefinition;
  labelTooltip: string;
  min: number;
  max: number;
  defaultValue: number;
  onSet: (value: number) => void;
  customNames?: string[];
};

function IntegerSelector({
  labelColor,
  labelIcon,
  labelTooltip,
  min,
  max,
  defaultValue,
  onSet,
  customNames
}: IntegerSelectorProps) {
  const inputRef = useRef<Input>(null);
  const [editing, setEditing] = useState(false);

  const onSubmit = useCallback(() => {
    let integer = parseInt(inputRef.current?.getValue());
    integer = Math.max(min, Math.min(max, integer));
    if (customNames) {
      inputRef.current?.setValue(customNames[integer]);
    } else {
      inputRef.current?.setValue(integer.toString());
    }
    setEditing(false);
    onSet(integer);
  }, [inputRef, min, max, onSet, customNames]);

  return (
    <div className={styles['feed-controls-row']}>
      <Label color={labelColor} tooltip={labelTooltip}>
        <FontAwesomeIcon icon={labelIcon} />
      </Label>
      <Input
        className={styles['input'] + (editing ? ' editing' : '')}
        defaultValue={customNames ? customNames[defaultValue] : defaultValue.toString()}
        ref={inputRef}
        onChange={(value) => {
          if (value.match(/\D/)) {
            value = value.replace(/\D/g, '');
            inputRef.current?.setValue(value);
          }
        }}
        onFocus={() => {
          setEditing(true);
          if (customNames) {
            inputRef.current?.setValue(defaultValue.toString());
          }
          inputRef.current?.selectAll();
        }}
        onBlur={onSubmit}
      />
      <Button onClick={onSubmit}>
        <FontAwesomeIcon icon={faCheck} />
      </Button>
    </div>
  );
}

type FeedProps = {
  feedIndex: number;
  max: number;
};

function Feed({ feedIndex, max }: FeedProps) {
  const style = getComputedStyle(document.body);
  const redBg = style.getPropertyValue('--red-background');
  const blueBg = style.getPropertyValue('--blue-background');

  const [rerenderCount, setRerenderCount] = useState(0);
  useEffect(() => {
    const update = () => setRerenderCount((c) => c + 1);
    window.addEventListener('feeds-updated', update);
    return () => {
      window.removeEventListener('feeds-updated', update);
    };
  }, []);

  return (
    <div className={styles['feed']}>
      <div className={styles['feed-controls-row']}>
        <div className={styles['icon']}>
          <FontAwesomeIcon icon={faDisplay} />
          <FontAwesomeIcon className={styles['icon-digit']} icon={[faCar, faCar, faFlask][feedIndex]} />
        </div>
      </div>
      <IntegerSelector
        labelColor={feedIndex < 2 ? redBg : blueBg}
        labelIcon={faCamera}
        labelTooltip='Camera (1-indexed)'
        min={1}
        max={max}
        defaultValue={feedCameras[feedIndex]}
        onSet={(value) => {
          feedCameras[feedIndex] = value;
          window.dispatchEvent(new Event('feeds-updated'));
        }}
        key={feedCameras[feedIndex]}
      />
    </div>
  );
}

export default function Feeds() {
  return (
    <div className={styles['feeds-panel']}>
      <div className={styles['feeds']}>
        <Feed feedIndex={0} max={6} />
        <Feed feedIndex={1} max={6} />
      </div>
      <div className={styles['feeds']}>
        <Feed feedIndex={2} max={3} />
      </div>
    </div>
  );
}
