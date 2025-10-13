import { getKeybind } from './keybinds';
import { settingsRef } from './refs';
import { ros } from './ros';
import { Topic } from 'roslib';

let feedCameras = [0, 0] as [number, number];

// Load feed config from local storage
const feedConfig = localStorage.getItem('feed-config');
if (feedConfig) {
  const feeds: any = JSON.parse(feedConfig);
  if (feeds.cameras?.length === 2 && feeds.cameras.every((c: any) => typeof c === 'number')) {
    feedCameras = feeds.cameras;
  }
}

// Emit feed-specific update events only
function emitFeedUpdated(feedIndex: 0 | 1) {
  const eventName = feedIndex === 0 ? 'feeds-updated-distobee' : 'feeds-updated-sieve';
  window.dispatchEvent(new CustomEvent(eventName, { detail: { camera: feedCameras[feedIndex] } }));
}

// Save feed config on specific events only
function saveFeedConfig() {
  localStorage.setItem('feed-config', JSON.stringify({ cameras: feedCameras } as any));
}
window.addEventListener('feeds-updated-distobee', saveFeedConfig as EventListener);
window.addEventListener('feeds-updated-sieve', saveFeedConfig as EventListener);

// ROS connection
window.addEventListener('ros-connect', () => {
  const topicDistobeeFeed = new Topic({
    ros: ros,
    name: '/set_feed/distobee',
    messageType: 'std_msgs/Int8'
  });

  const topicSieveFeed = new Topic({
    ros: ros,
    name: '/set_feed/sieve',
    messageType: 'std_msgs/Int8'
  });

  // Publish camera change to the correct feed topic
  const publishFeed = (feedIndex: 0 | 1) => {
    const camera = feedCameras[feedIndex];
    const msg = { data: camera };
    if (feedIndex === 0) {
      topicDistobeeFeed.publish(msg);
    } else {
      topicSieveFeed.publish(msg);
    }
  };

  // Listen for separate update events → publish only to the target topic
  window.addEventListener('feeds-updated-distobee', () => publishFeed(0));
  window.addEventListener('feeds-updated-sieve', () => publishFeed(1));

  // Optional: send initial state to both topics
  publishFeed(0);
  publishFeed(1);
});

// Handle keyboard shortcuts
function showCameraOnFeed(camera: number, feedIndex: 0 | 1) {
  feedCameras[feedIndex] = camera;
  emitFeedUpdated(feedIndex);
}

window.addEventListener('keydown', (event) => {
  // Skip if input field is focused or settings are open
  if ((document.activeElement as HTMLElement)?.tagName === 'INPUT') return;
  if (settingsRef.current?.isShown()) return;

  switch (event.code) {
    // Distobee feed (0)
    case getKeybind('Show Camera 1 on Distobee feed'):
      showCameraOnFeed(1, 0);
      break;
    case getKeybind('Show Camera 2 on Distobee feed'):
      showCameraOnFeed(2, 0);
      break;
    case getKeybind('Show Camera 3 on Distobee feed'):
      showCameraOnFeed(3, 0);
      break;
    case getKeybind('Show Camera 4 on Distobee feed'):
      showCameraOnFeed(4, 0);
      break;
    case getKeybind('Show Camera 5 on Distobee feed'):
      showCameraOnFeed(5, 0);
      break;
    case getKeybind('Show Camera 6 on Distobee feed'):
      showCameraOnFeed(6, 0);
      break;

    // Sieve feed (1)
    case getKeybind('Show Camera 1 on Sieve feed'):
      showCameraOnFeed(1, 1);
      break;
    case getKeybind('Show Camera 2 on Sieve feed'):
      showCameraOnFeed(2, 1);
      break;
    case getKeybind('Show Camera 3 on Sieve feed'):
      showCameraOnFeed(3, 1);
      break;
  }
});

export { feedCameras };
