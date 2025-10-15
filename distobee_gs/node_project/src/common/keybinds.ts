const defaultKeybinds = {
  'Show Camera 1 on Distobee Main Feed': 'Digit1',
  'Show Camera 2 on Distobee Main Feed': 'Digit2',
  'Show Camera 3 on Distobee Main Feed': 'Digit3',
  'Show Camera 4 on Distobee Main Feed': 'Digit4',
  'Show Camera 5 on Distobee Main Feed': 'Digit5',
  'Show Camera 6 on Distobee Main Feed': 'Digit6',
  'Show Camera 1 on Distobee Alt Feed': 'KeyQ',
  'Show Camera 2 on Distobee Alt Feed': 'KeyW',
  'Show Camera 3 on Distobee Alt Feed': 'KeyE',
  'Show Camera 4 on Distobee Alt Feed': 'KeyR',
  'Show Camera 5 on Distobee Alt Feed': 'KeyT',
  'Show Camera 6 on Distobee Alt Feed': 'KeyY',
  'Show Camera 1 on Sifter Feed': 'KeyA',
  'Show Camera 2 on Sifter Feed': 'KeyS',
  'Show Camera 3 on Sifter Feed': 'KeyD',
};

// Copy default keybinds to keybinds.
let keybinds = Object.assign({}, defaultKeybinds);

// Load state from local storage if available.
const savedKeybinds = localStorage.getItem('keybinds');
if (savedKeybinds) {
  // keybinds = JSON.parse(savedKeybinds);
  // Object.assign in order to keep any new keybinds from defaultKeybinds.
  keybinds = Object.assign(keybinds, JSON.parse(savedKeybinds));
  // Remove keybinds that are not in defaultKeybinds.
  for (const key in keybinds) {
    if (!(key in defaultKeybinds)) {
      delete keybinds[key];
    }
  }
}

function getKeybind(action: string): string {
  if (keybinds[action] === undefined) {
    throw new Error(`Keybind for action "${action}" not found.`);
  }
  return keybinds[action];
}

function setKeybind(action: string, code: string) {
  keybinds[action] = code;
  localStorage.setItem('keybinds', JSON.stringify(keybinds));
}

function resetKeybind(action: string): void {
  keybinds[action] = defaultKeybinds[action];
  localStorage.setItem('keybinds', JSON.stringify(keybinds));
}

function resetAllKeybinds(): void {
  keybinds = Object.assign({}, defaultKeybinds);
  localStorage.setItem('keybinds', JSON.stringify(keybinds));
}

export { getKeybind, setKeybind, resetKeybind, resetAllKeybinds, keybinds };
