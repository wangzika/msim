// Input handling is independent of WebGL so it can be tested without a GPU.
export function movementKey(value) {
  const arrows={ArrowUp:'w',ArrowDown:'s',ArrowLeft:'a',ArrowRight:'d'};
  const key=arrows[value] || value.toLowerCase();
  return key.length===1 && 'wasdrfqe'.includes(key) ? key : null;
}

export class FlightInput {
  constructor() {this.sources=new Map();}
  press(source,key) {this.sources.set(source,key);}
  release(source) {return this.sources.delete(source);}
  clear() {this.sources.clear();}
  keys() {return [...new Set(this.sources.values())].sort();}
}

export class CommandQueue {
  constructor(send,onError=()=>{}) {this.send=send;this.onError=onError;this.pending=[];this.running=false;}
  push(value) {
    const movement=command=>command && ['keys','hold'].includes(command.op);
    if (movement(value) && movement(this.pending[this.pending.length-1])) this.pending[this.pending.length-1]=value;
    else this.pending.push(value);
    return this.drain();
  }
  async drain() {
    if (this.running) return;
    this.running=true;
    try {
      while (this.pending.length) {
        const value=this.pending.shift();
        try {await this.send(value);}
        catch(error) {this.pending=[];this.onError(error);}
      }
    } finally {this.running=false;}
  }
}
