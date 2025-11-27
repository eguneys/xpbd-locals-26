export function Loop(update: (dt: number) => void, render: (alpha: number) => void, after_render?: () => void) {

  const timestep = 1000/60
  let last_time = performance.now()
  let accumulator = 0

  function step(current_time: number) {
    requestAnimationFrame(step)


    let delta_time = Math.min(current_time - last_time, 25)
    last_time = current_time

    accumulator += delta_time

    while (accumulator >= timestep) {
      update(timestep)
      accumulator -= timestep
    }

    render(accumulator / timestep)

    after_render?.()
  }
  requestAnimationFrame(step)
}

type XY = [number, number]
export type TouchMouse = {
  on_up(n: XY): void
  on_down(n: XY): void
  on_move(n: XY): void
}

export function TouchMouse(el: HTMLElement, hooks: TouchMouse) {

  const eventPosition = (e: PointerEvent): [number, number] | undefined => {
    if (e.clientX || e.clientX === 0) return [e.clientX, e.clientY!];
  };

  const normalized = (e: PointerEvent): [number, number] => {
    let [x, y] = eventPosition(e) ?? [0, 0]

    return [(x - bounds.left) / bounds.width, (y - bounds.top) / bounds.height]
  }

  function on_down(ev: PointerEvent) {
    el.setPointerCapture(ev.pointerId)
    let p = normalized(ev)
    hooks.on_down(p)
  }

  function on_up(ev: PointerEvent) {
    let p = normalized(ev)
    hooks.on_up(p)
  }

  function on_move(ev: PointerEvent) {
    let p = normalized(ev)
    hooks.on_move(p)
  }
  
  let cx = (el as HTMLCanvasElement).getContext('2d')!
  let bounds: DOMRect
  on_resize()


  function on_resize() {
    bounds = el.getBoundingClientRect()
    cx.imageSmoothingEnabled = false
  }

  function on_scroll() {
    on_resize()
  }

  el.addEventListener('pointerdown', on_down, { passive: false })
  el.addEventListener('pointermove', on_move, { passive: false })
  document.addEventListener('pointerup', on_up)


  new ResizeObserver(on_resize).observe(el)
  document.addEventListener('scroll', on_scroll, { capture: true, passive: true })
  window.addEventListener('resize', on_scroll, { passive: true })
}