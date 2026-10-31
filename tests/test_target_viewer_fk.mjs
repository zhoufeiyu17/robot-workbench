import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {modelFrames} from '../web/target_viewer.js';
const [modelPath,casesPath]=process.argv.slice(2);
assert(modelPath&&casesPath,'provide metadata and Python native FK cases');
const model=JSON.parse(await readFile(modelPath)),cases=JSON.parse(await readFile(casesPath));let peak=0,count=0;
for(const row of cases){
  const actual=modelFrames(model,row.q,row.gaps);
  for(const [name,expected]of Object.entries(row.frames)){
    const matrix=actual.get(name);assert(matrix,'missing link '+name);
    const values=matrix.clone().transpose().toArray();
    expected.flat().forEach((v,i)=>peak=Math.max(peak,Math.abs(v-values[i])));count++;
  }
}
assert(peak<1e-9,'browser/native FK differs '+peak);
assert.throws(()=>modelFrames(model,Array(12).fill(NaN),[83,83]),/finite/);
const limits=model.jaw_limits_mm||[[0,83],[0,83]];
assert.throws(()=>modelFrames(model,Array(12).fill(0),[limits[0][1]+1,limits[1][1]]),/device-bounded/);
const arx13={...model,jaw_limits_mm:[[0,80],[0,89]]};
assert.doesNotThrow(()=>modelFrames(arx13,Array(12).fill(0),[80,89]));
assert.throws(()=>modelFrames(arx13,Array(12).fill(0),[81,89]),/device-bounded/);
assert.throws(()=>modelFrames(arx13,Array(12).fill(0),[80,90]),/device-bounded/);
console.log(JSON.stringify({passed:true,cases:cases.length,link_transforms:count,max_absolute_error:peak,scope:'actual URDF native Python FK vs browser Three.js FK; no device IO'}));
