import assert from 'assert';
import fs from 'fs';

async function main() {
  const source=fs.readFileSync(new URL('../../web/sim-console/flight-input.js',import.meta.url),'utf8');
  const {movementKey,FlightInput,CommandQueue}=await import(`data:text/javascript;base64,${Buffer.from(source).toString('base64')}`);
  assert.deepStrictEqual(['ArrowUp','ArrowDown','ArrowLeft','ArrowRight','W','r','Escape'].map(movementKey),['w','s','a','d','w','r',null]);
  const input=new FlightInput();input.press('KeyW','w');input.press('ArrowUp','w');
  input.release('ArrowUp');assert.deepStrictEqual(input.keys(),['w']);
  input.press('pointer:1','d');input.release('KeyW');assert.deepStrictEqual(input.keys(),['d']);
  assert.strictEqual(input.release('KeyW'),false);input.clear();assert.deepStrictEqual(input.keys(),[]);
  const sent=[];let unlock,inFlight=0,maximum=0;
  const queue=new CommandQueue(async value=>{
    inFlight++;maximum=Math.max(maximum,inFlight);sent.push(value);
    if (sent.length===1) await new Promise(resolve=>{unlock=resolve;});
    inFlight--;
  });
  const done=queue.push({op:'keys',keys:['w']});
  queue.push({op:'keys',keys:['d']});queue.push({op:'hold'});
  queue.push({op:'takeoff'});queue.push({op:'keys',keys:['a']});
  unlock();await done;
  assert.strictEqual(maximum,1);
  assert.deepStrictEqual(sent,[{op:'keys',keys:['w']},{op:'hold'},{op:'takeoff'},{op:'keys',keys:['a']}]);
  let failure;const broken=new CommandQueue(async()=>{throw new Error('offline');},error=>{failure=error;});
  await broken.push({op:'hold'});assert.strictEqual(failure.message,'offline');
  console.log('Web flight input: arrow aliases, held-key sources, ordered/coalesced commands and transport failure passed');
}
main().catch(error=>{console.error(error);process.exitCode=1;});
