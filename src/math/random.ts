/* https://stackoverflow.com/questions/521295/seeding-the-random-number-generator-in-javascript */
export const make_random = (seed = 0) => {
  return () => {
    var x = Math.sin(seed++) * 10000;
    return x - Math.floor(x);
  }
}

export type RNG = () => number

export const random = make_random()

export function rnd_h(rng: RNG = random) {
  return rng() * 2 - 1
}

export function rnd_int_h(max: number, rng: RNG = random) {
  return rnd_h(rng) * max
}

export function rnd_sign(rng: RNG = random) {
  return rng() < 0.5 ? -1 : 1  
}

export function rnd_sint(min: number, max: number, rng: RNG = random) {
    return rnd_sign(rng) * rnd_int(min, max, rng)
}

export function rnd_int(min: number, max: number, rng: RNG = random) {
    return Math.floor(rng() * (max - min) + min)
}

export function rnd_float(min: number, max: number, rng: RNG = random) {
    return (rng() * (max - min) + min)
}


export function arr_rnd<A>(arr: Array<A>) {
  return arr[rnd_int(0, arr.length)]
}

export function arr_remove<A>(arr: Array<A>, a: A) {
  arr.splice(arr.indexOf(a), 1)
}

export function arr_shuffle<A>(array: Array<A>) {
  let currentIndex = array.length;

  // While there remain elements to shuffle...
  while (currentIndex != 0) {

    // Pick a remaining element...
    let randomIndex = Math.floor(Math.random() * currentIndex);
    currentIndex--;

    // And swap it with the current element.
    [array[currentIndex], array[randomIndex]] = [
      array[randomIndex], array[currentIndex]];
  }
  return array
}

