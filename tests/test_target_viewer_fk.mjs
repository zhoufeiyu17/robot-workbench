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
assert.throws(()=>modelFrames(model,Array(12).fill(0),[84,83]),/0–83/);
console.log(JSON.stringify({passed:true,cases:cases.length,link_transforms:count,max_absolute_error:peak,scope:'actual URDF native Python FK vs browser Three.js FK; no device IO'}));
