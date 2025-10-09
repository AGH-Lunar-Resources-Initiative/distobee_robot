// Pipes.tsx
import styles from './pipes.module.css'
import { useEffect, useState } from 'react'
import { ros } from '../common/ros'
import { Topic } from 'roslib'
import pipeUrl from '!!url-loader!../media/distobee-pipe.svg'
import Label from '../components/label'
import Input from '../components/input'

export type PipeStates = { left?: boolean; right?: boolean }
let lastPipeStates: PipeStates | null = null

window.addEventListener('ros-connect', () => {
  const topic = new Topic({
    ros,
    name: '/pipe_states',
    messageType: 'distobee_interfaces/PipeStates'
  })
  topic.subscribe((msg: PipeStates) => {
    lastPipeStates = msg
    window.dispatchEvent(new Event('pipe-states'))
  })
})

export default function Pipes() {
  const [leftAlert, setLeftAlert] = useState(false)
  const [rightAlert, setRightAlert] = useState(false)

  const setPipeStates = () => {
    setLeftAlert(Boolean(lastPipeStates?.left))
    setRightAlert(Boolean(lastPipeStates?.right))
  }

  useEffect(() => {
    window.addEventListener('pipe-states', setPipeStates)
    return () => window.removeEventListener('pipe-states', setPipeStates)
  }, [])

  const maskStyle = {
    WebkitMask: `url(${pipeUrl}) center / contain no-repeat`,
    mask: `url(${pipeUrl}) center / contain no-repeat`
  } as const

  return (
    <div className={styles.pipes}>
      <div className={styles.wrapper}>
        <div className={`${styles.col} ${leftAlert ? styles.alert : styles.ok}`}>
          <div className={styles['feed-controls-row']}>
            <Label>Left</Label>
            <Input className={styles.input} defaultValue="Left" disabled />
          </div>
          <div className={styles['pipe-state']}>
            <div className={styles.icon} style={maskStyle} />
          </div>
        </div>

        <div className={`${styles.col} ${rightAlert ? styles.alert : styles.ok}`}>
          <div className={styles['feed-controls-row']}>
            <Label>Right</Label>
            <Input className={styles.input} defaultValue="Right" disabled />
          </div>
          <div className={styles['pipe-state']}>
            <div className={styles.icon} style={maskStyle} />
          </div>
        </div>
      </div>
    </div>
  )
}
