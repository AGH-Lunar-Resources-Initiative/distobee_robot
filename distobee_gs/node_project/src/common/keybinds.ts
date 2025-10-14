const defaultKeybinds = {
  // Feeds
  'Show Camera 1 on Distobee feed': 'Digit1',
  'Show Camera 2 on Distobee feed': 'Digit2',
  'Show Camera 3 on Distobee feed': 'Digit3',
  'Show Camera 4 on Distobee feed': 'Digit4',
  'Show Camera 5 on Distobee feed': 'Digit5',
  'Show Camera 6 on Distobee feed': 'Digit6',
  'Show Camera 1 on Sieve feed': 'Digit7',
  'Show Camera 2 on Sieve feed': 'Digit8',
  'Show Camera 3 on Sieve feed': 'Digit9',
  // Other
  'Engage Remote E-STOP': 'Semicolon'
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
