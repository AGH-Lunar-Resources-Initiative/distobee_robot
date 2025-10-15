import { getKeybind } from './keybinds';
import { alertsRef, settingsRef } from './refs';
import { ros } from './ros';
import { Topic } from 'roslib';

let feedCameras = [0, 0, 0] as [number, number, number];

// Load feed config from local storage
const feedConfig = localStorage.getItem('feed-config');
if (feedConfig) {
  const feeds: any = JSON.parse(feedConfig);
  if (feeds.cameras?.length === 3 && feeds.cameras.every((c: any) => typeof c === 'number')) {
    feedCameras = feeds.cameras;
  }
}

// Save feed config to local storage on update.
window.addEventListener('feeds-updated', () => {
  localStorage.setItem(
    'feed-config',
    JSON.stringify({
      cameras: feedCameras,
    } as any)
  );
});

// ROS connection
window.addEventListener('ros-connect', () => {
  const topicDistobeeMainFeed = new Topic({
    ros: ros,
    name: '/set_feed/distobee_main',
    messageType: 'std_msgs/Int8'
  });
  const topicDistobeeAltFeed = new Topic({
    ros: ros,
    name: '/set_feed/distobee_alt',
    messageType: 'std_msgs/Int8'
  });
  const topicSifterFeed = new Topic({
    ros: ros,
    name: '/set_feed/sifter',
    messageType: 'std_msgs/Int8'
  });
  const topics = [topicDistobeeMainFeed, topicDistobeeAltFeed, topicSifterFeed];

  // Keep track of previous values to avoid unnecessary calls.
  // 0 is an invalid state that will force update on first call.
  let prevCameras = [0, 0, 0];
  const publishSetFeeds = () => {
    for (let feed = 0; feed < 3; feed++) {
      if (feedCameras[feed] === prevCameras[feed]) {
        continue; // No change, skip to next feed.
      }
      // Remember the current values for the next call.
      prevCameras[feed] = feedCameras[feed];

      topics[feed].publish({ data: feedCameras[feed] });
    }
  };

  // Listen for separate update events -> publish only to the target topic
  window.addEventListener('feeds-updated', () => publishSetFeeds());
  publishSetFeeds();
});

// Handle keyboard shortcuts
function showCameraOnFeed(camera: number, feedIndex: 0 | 1 | 2) {
  feedCameras[feedIndex] = camera;
  window.dispatchEvent(new Event('feeds-updated'));
}

window.addEventListener('keydown', (event) => {
  // Skip if input field is focused or settings are open
  if ((document.activeElement as HTMLElement)?.tagName === 'INPUT') return;
  if (settingsRef.current?.isShown()) return;

  switch (event.code) {
    // Distobee main feed (0)
    case getKeybind('Show Camera 1 on Distobee Main Feed'):
      showCameraOnFeed(1, 0);
      break;
    case getKeybind('Show Camera 2 on Distobee Main Feed'):
      showCameraOnFeed(2, 0);
      break;
    case getKeybind('Show Camera 3 on Distobee Main Feed'):
      showCameraOnFeed(3, 0);
      break;
    case getKeybind('Show Camera 4 on Distobee Main Feed'):
      showCameraOnFeed(4, 0);
      break;
    case getKeybind('Show Camera 5 on Distobee Main Feed'):
      showCameraOnFeed(5, 0);
      break;
    case getKeybind('Show Camera 6 on Distobee Main Feed'):
      showCameraOnFeed(6, 0);
      break;

    // Distobee alt feed (1)
    case getKeybind('Show Camera 1 on Distobee Alt Feed'):
      showCameraOnFeed(1, 1);
      break;
    case getKeybind('Show Camera 2 on Distobee Alt Feed'):
      showCameraOnFeed(2, 1);
      break;
    case getKeybind('Show Camera 3 on Distobee Alt Feed'):
      showCameraOnFeed(3, 1);
      break;
    case getKeybind('Show Camera 4 on Distobee Alt Feed'):
      showCameraOnFeed(4, 1);
      break;
    case getKeybind('Show Camera 5 on Distobee Alt Feed'):
      showCameraOnFeed(5, 1);
      break;
    case getKeybind('Show Camera 6 on Distobee Alt Feed'):
      showCameraOnFeed(6, 1);
      break;

    // Sifter feed (2)
    case getKeybind('Show Camera 1 on Sifter Feed'):
      showCameraOnFeed(1, 2);
      break;
    case getKeybind('Show Camera 2 on Sifter Feed'):
      showCameraOnFeed(2, 2);
      break;
    case getKeybind('Show Camera 3 on Sifter Feed'):
      showCameraOnFeed(3, 2);
      break;
  }
});

export { feedCameras };
