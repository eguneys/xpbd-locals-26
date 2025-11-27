// 1. Define the Actions your game supports
// Use a const object + exported type so the symbol is erasable
// in environments that only allow type-level syntax (no runtime enums).
export const GameAction = {
  LEFT: "LEFT",
  RIGHT: "RIGHT",
  UP: "UP",
  DOWN: "DOWN",
  JUMP: "JUMP",
  ATTACK: "ATTACK",
} as const;

export type GameAction = typeof GameAction[keyof typeof GameAction];

// 2. Define the bindings (Default controls)
type KeyBinding = Record<string, GameAction>;

const DEFAULT_BINDINGS: KeyBinding = {
  "ArrowLeft": GameAction.LEFT,
  "KeyA": GameAction.LEFT,
  "ArrowRight": GameAction.RIGHT,
  "KeyD": GameAction.RIGHT,
  "ArrowUp": GameAction.UP,
  "KeyW": GameAction.UP,
  "ArrowDown": GameAction.DOWN,
  "KeyS": GameAction.DOWN,
  "Space": GameAction.JUMP,
  "KeyK": GameAction.JUMP,
  "KeyJ": GameAction.ATTACK
};

export class InputController {
  // Current state of keys (True = Down, False = Up)
  private currentKeys: Set<string> = new Set();
  
  // State of keys in the previous frame (Used for edge detection)
  private previousKeys: Set<string> = new Set();
  
  // Mapping of physical keys to game actions
  private bindings: KeyBinding;

  constructor(bindings: KeyBinding = DEFAULT_BINDINGS) {
    this.bindings = bindings;
    
    // Attach event listeners
    window.addEventListener("keydown", this.onKeyDown);
    window.addEventListener("keyup", this.onKeyUp);
    // Safety: clear keys if window loses focus
    window.addEventListener("blur", this.reset); 
  }

  // --- Event Handlers ---

  private onKeyDown = (event: KeyboardEvent) => {
    // Prevent default scrolling for game keys (Space, Arrows)
    if (this.bindings[event.code]) {
      // event.preventDefault(); // Uncomment if you want to stop browser scrolling
      this.currentKeys.add(event.code);
    }
  }

  private onKeyUp = (event: KeyboardEvent) => {
    if (this.bindings[event.code]) {
      this.currentKeys.delete(event.code);
    }
  }

  private reset = () => {
    this.currentKeys.clear();
    this.previousKeys.clear();
  }

  // --- The Game Loop Hook ---
  
  // Call this at the VERY START of every game frame
  public update() {
    // Copy current state to previous state for the next frame's comparison
    this.previousKeys = new Set(this.currentKeys);
  }

  // --- Public API for the Player ---

  /**
   * Returns TRUE while the key is held down.
   * Good for: Movement (Walking/Running)
   */
  public isDown(action: GameAction): boolean {
    return this.checkAction(action, (key) => this.currentKeys.has(key));
  }

  /**
   * Returns TRUE only on the frame the key was pressed.
   * Good for: Jumping, Shooting, UI confirmation
   */
  public wasPressed(action: GameAction): boolean {
    return this.checkAction(action, (key) => 
      this.currentKeys.has(key) && !this.previousKeys.has(key)
    );
  }

  /**
   * Returns TRUE only on the frame the key was released.
   * Good for: Variable jump height (releasing space early)
   */
  public wasReleased(action: GameAction): boolean {
    return this.checkAction(action, (key) => 
      !this.currentKeys.has(key) && this.previousKeys.has(key)
    );
  }

  // Helper to check if ANY valid key for an action meets the criteria
  private checkAction(action: GameAction, predicate: (key: string) => boolean): boolean {
    for (const [key, boundAction] of Object.entries(this.bindings)) {
      if (boundAction === action && predicate(key)) {
        return true;
      }
    }
    return false;
  }
  
  // Clean up listeners when scene is destroyed
  public destroy() {
      window.removeEventListener("keydown", this.onKeyDown);
      window.removeEventListener("keyup", this.onKeyUp);
      window.removeEventListener("blur", this.reset);
  }
}