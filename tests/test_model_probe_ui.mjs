import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
const source=await readFile(new URL('../web/model_probe.js',import.meta.url),'utf8');
const app=await readFile(new URL('../web/app.js',import.meta.url),'utf8');
assert(!source.includes('/api/actions/'));assert(!source.includes('/api/operations/action'));
assert(source.includes('/api/triton/infer'));assert(source.includes('已保存样本，不执行机器人动作'));
assert(app.includes("const modelError=model.status==='error'&&!modelOperation,"));
console.log('Model-only probes have no robot action route; not_loaded is not a failed model.');
