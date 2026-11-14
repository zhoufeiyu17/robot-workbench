import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {recordingPolicyFor, recordingCapacity, recordingSvg} from '../web/architecture/recording.mjs';
import {relatedCases,changeScope,useCaseSvg,operationsSvg} from '../web/architecture/usecases.mjs';
import {buildDraft, validateDraft, escapeHtml, componentSvg, flowSvg, modelExtensionHtml, modelExtensionSelection, modelExtensionSvg} from '../web/architecture/app.mjs';

const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
const catalog=JSON.parse(fs.readFileSync(path.join(root,'web/architecture/catalog.json'),'utf8'));
assert.equal(catalog.schema_version,2);
assert.equal(catalog.components.length,6);
assert.equal(catalog.components.filter(row=>row.id==='evidence').length,1);
assert(!catalog.supporting.some(row=>row.id==='evidence'));
const componentIds=new Set([...catalog.components,...catalog.supporting].map(c=>c.id));
const ruleIds=new Set(catalog.gates.map(g=>g.id));
assert.equal(ruleIds.size,catalog.gates.length,'duplicate rule ID');
assert.equal(new Set(catalog.interfaces.map(i=>i.id)).size,catalog.interfaces.length);
for(const component of catalog.components) {
  assert(catalog.statuses[component.status]);
  assert(component.responsibilities.length&&component.acceptance.length);
}
for(const row of [...catalog.gates,...catalog.interfaces,...catalog.components]) {
  assert(row.sources.length,'missing source');
  for(const id of row.sources)assert(catalog.sources[id],`unknown source ${id}`);
}
for(const rule of catalog.gates) {
  assert(componentIds.has(rule.site));
  assert(rule.current&&rule.action&&rule.policy_key&&rule.phases.length&&rule.profile);
}
for(const contract of catalog.interfaces) {
  assert(componentIds.has(contract.producer)&&componentIds.has(contract.consumer));
  assert(contract.clock&&contract.units&&contract.failure&&contract.fields.length);
}
for(const source of Object.values(catalog.sources)) {
  assert(source.line>0&&/^[a-f0-9]{64}$/.test(source.sha256));
  assert(source.excerpt.includes(String(source.line)));
}
const defaults=Object.fromEntries(catalog.parameters.map(p=>[p.key,p.value]));
for(const p of catalog.parameters)assert(ruleIds.has(p.source));
assert.deepEqual(validateDraft(defaults,catalog.parameters),[]);
for(const steps of [1,15,25,30,60]) {
  const values={...defaults,steps};
  const draft=buildDraft(values,{tracking:'disabled',arrival:'disabled',pairing:'observe',recordingPolicy:recordingPolicyFor(catalog,'infer_debug')},catalog.revision,catalog.execution_contract);
  assert.equal(draft.execution.source_duration_s,steps/30);
  assert.equal(draft.execution.max_segment_duration_s,steps/30*4);
  assert.equal(draft.status,'draft_for_review');
  assert.equal(draft.tracking.mode,'disabled');
  assert.equal(draft.arrival.mode,'disabled');
  assert.equal(draft.recording.failure_action,'continue_and_mark_incomplete');
  assert.equal(draft.measured.joint_velocity_rad_s,2.5);
}
for(const steps of [0,61,1.5,NaN,Infinity])assert(validateDraft({...defaults,steps},catalog.parameters).length);
assert(validateDraft({...defaults,planning_speed:2,measured_speed:1.5},catalog.parameters).length);
assert(validateDraft({...defaults,result_window:0},catalog.parameters).length);
assert.equal(escapeHtml('<img onerror="x">'),'&lt;img onerror=&quot;x&quot;&gt;');
const html=fs.readFileSync(path.join(root,'web/architecture/index.html'),'utf8');
const extension=catalog.model_extensions.find(row=>row.id==='wam-e03');
assert.equal(extension.candidate.horizon,32);assert.equal(extension.candidate.denoise_steps,2);
assert.equal(typeof extension.source.gpu_executed,'boolean');
assert(extension.cases.length>0);assert.equal(new Set(extension.cases.map(row=>row.id)).size,extension.cases.length);
assert.deepEqual(extension.backend_design.families,{pi:['torch','triton'],wam:['torch','anyinfer']});
assert.equal(extension.backend_design.labels.anyinfer,'SO 算子加速');
assert(extension.cases.every(row=>['defined_not_run','cpu_partial','verified'].includes(row.state)));
for(const c of extension.cases)for(const id of c.use_cases)assert(catalog.use_cases.some(u=>u.id===id));
for(const steps of [1,15,32]) {
  const selection=modelExtensionSelection(extension,'wam',steps);
  assert.equal(selection.maximumSeconds,steps/30*4);assert.equal(selection.horizon,32);
}
assert.throws(()=>modelExtensionSelection(extension,'wam',33));
assert.equal(modelExtensionSelection(extension,'pi05',60).horizon,60);
assert.throws(()=>modelExtensionSelection(extension,'pi05',61));
const extensionHtml=modelExtensionHtml(extension);
assert(extensionHtml.includes('上限 2.5 秒'));
assert(modelExtensionHtml(extension,'pi05').includes('上限 0.5 秒'));

assert(extensionHtml.includes('max="32"')&&extensionHtml.includes('仅设计预览'));
assert(modelExtensionHtml(extension,'pi05').includes('max="60"'));
assert(modelExtensionSvg(extension).includes('WA5'));
const js=fs.readFileSync(path.join(root,'web/architecture/app.mjs'),'utf8')+'\n'+fs.readFileSync(path.join(root,'web/architecture/usecases.mjs'),'utf8')+'\n'+fs.readFileSync(path.join(root,'web/architecture/recording.mjs'),'utf8');
const ids=[...(html+extensionHtml).matchAll(/\bid="([^"]+)"/g)].map(m=>m[1]);
assert.equal(new Set(ids).size,ids.length,'duplicate DOM id');
for(const match of js.matchAll(/(?<!\.)get\('([^']+)'\)/g))assert(ids.includes(match[1]),`missing DOM id ${match[1]}`);
assert.equal((js.match(/fetch\(/g)||[]).length,1,'architecture page should only fetch its catalog');
assert(!js.includes('/api/actions')&&!js.includes('/prepare')&&!js.includes('/infer'));
assert(html.includes('实际应用入口 · 待开发'));
assert.equal(catalog.baseline.latest_run.execution_settings.settings.speed_protection.limit_rad_s,1.5);
assert.equal(catalog.baseline.latest_run.execution_settings.settings.arrival_enabled,false);
for(const target of[true,false]) {
  const svg=componentSvg(catalog,target);
  for(const id of catalog.components.map(c=>c.id))assert(svg.includes(`data-component="${id}"`));
}
assert(flowSvg().includes('GuardDecision'));
console.log(`PASS architecture catalog, ${catalog.gates.length} rules, ${catalog.interfaces.length} contracts, draft bounds/derived timing, source links, DOM IDs and isolated fetch surface`);

const caseIds=new Set(catalog.use_cases.map(row=>row.id));
const goalIds=new Set(catalog.goals.map(row=>row.id));
const interfaceIds=new Set(catalog.interfaces.map(row=>row.id));
const sharedIds=new Set(catalog.shared_steps.map(row=>row.id));
const actorIds=new Set(catalog.actors.map(row=>row.id));
assert.equal(caseIds.size,catalog.use_cases.length);
assert.equal(catalog.use_cases.length,22);
for(const row of catalog.use_cases) {
  assert(goalIds.has(row.goal)&&actorIds.has(row.actor));
  assert(row.preconditions.length&&row.main_flow.length&&row.success_postconditions.length&&row.effects.length);
  assert(row.alternatives.every(branch=>branch.trigger&&branch.resulting_state&&branch.next_action));
  assert(row.acceptance.some(test=>test.kind==='success')&&row.acceptance.some(test=>test.kind==='failure'));
  assert(row.acceptance.every(test=>test.given&&test.when&&test.then));
  for(const id of row.modules)assert(componentIds.has(id));
  for(const id of row.interfaces)assert(interfaceIds.has(id));
  for(const id of row.rules)assert(ruleIds.has(id));
  for(const id of row.shared_steps)assert(sharedIds.has(id));
  for(const id of row.evidence)assert(catalog.sources[id]);
}
for(const goal of catalog.goals)for(const id of goal.cases)assert(caseIds.has(id));
for(const journey of catalog.journeys)for(const id of journey.cases)assert(caseIds.has(id));
for(const id of componentIds)assert(relatedCases(catalog,id).length,`module without use case ${id}`);
for(const id of ruleIds)assert(catalog.use_cases.some(row=>row.rules.includes(id)),`rule without use case ${id}`);
for(const id of interfaceIds)assert(catalog.use_cases.some(row=>row.interfaces.includes(id)),`interface without use case ${id}`);
for(const relation of catalog.use_case_relations)assert(caseIds.has(relation.from)&&caseIds.has(relation.to));
const states=new Set(catalog.operation_design.states.map(row=>row.id));
for(const [from,to]of catalog.operation_design.transitions) {
  assert(states.has(from)&&states.has(to));
  assert(catalog.operation_design.transition_reasons[from+'>'+to]);
}
assert(catalog.operation_design.transitions.some(([from,to])=>from==='unknown'&&to==='verifying'));
assert(!catalog.operation_design.transitions.some(([from,to])=>from==='unknown'&&to==='running'));
for(const change of catalog.change_examples) {
  const scope=changeScope(catalog,change.id);
  assert.equal(scope.useCases.length,change.cases.length);
  assert.equal(scope.validation.length,change.tiers.length);
  for(const id of change.interfaces)assert(interfaceIds.has(id));
  for(const id of change.rules)assert(ruleIds.has(id));
}
assert.deepEqual(changeScope(catalog,'C1').tiers,['V0']);
assert(changeScope(catalog,'C3').tiers.includes('V4'));
assert.throws(()=>changeScope(catalog,'missing'));
for(const group of [...new Set(catalog.use_cases.map(row=>row.group))]) {
  const svg=useCaseSvg(catalog,group,group==='运维'?'UC09':'UC05');
  assert(svg.includes('步骤引用'));
  for(const row of catalog.use_cases.filter(row=>row.group===group))assert(svg.includes(`data-use-case="${row.id}"`));
}
assert(operationsSvg().includes('unknown'));
console.log(`PASS ${catalog.use_cases.length} use cases, ${catalog.journeys.length} journeys, ${catalog.operation_design.transitions.length} operation transitions, full module/interface/rule traceability and impact-based validation plans`);

for(const contract of catalog.interfaces) {
  for(const id of contract.producers||[])assert(componentIds.has(id));
  for(const id of contract.consumers||[])assert(componentIds.has(id));
}
assert.equal(catalog.recording_design.approved_fault_policy.status,'user_confirmed');
for(const purpose of ['calibration','rule_collection','control_research_measurement']) {
  const policy=recordingPolicyFor(catalog,purpose);
  assert.equal(policy.failure_profile,'data_required');
  assert.equal(policy.require_ready_before_start,true);
  assert.equal(policy.failure_action,'request_controlled_end');
}
assert.equal(recordingPolicyFor(catalog,'infer_debug').require_ready_before_start,false);
assert.throws(()=>recordingPolicyFor(catalog,'unknown'));
const capacity=recordingCapacity(catalog.recording_design.capacity_example);
assert.equal(capacity.rawBytesPerSecond,82944000);
assert.equal(capacity.bufferMiB,158.203125);
assert(recordingSvg().includes('独立进程'));
for(const stream of catalog.recording_design.streams)assert(interfaceIds.has(stream.contract));
assert(catalog.use_cases.find(row=>row.id==='UC05').shared_steps.includes('SC05'));
assert(catalog.use_cases.find(row=>row.id==='UC05').shared_steps.includes('SC06'));
console.log('PASS five-module recorder design, cross-business capture interfaces, confirmed purpose policies, independent data flow and capacity arithmetic');
