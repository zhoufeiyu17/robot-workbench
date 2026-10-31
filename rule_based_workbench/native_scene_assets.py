"""Prepare an isolated native consumer variant; never edit the shared source.

The existing native release predates object selection. These exact, checked
edits route its three cup consumers through the shared asset binding. Existing
collision, contact and acceptance algorithms remain unchanged.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

PACKAGE = 'source/robot_sim/robot_sim/tasks/manager_based/coffee_make/'
OLD_CUP = '"objects/anyverse_lidded_paper_cup_9cm_13cm/v1"'


def edits():
    task_edits = {
        'episode_planner.py': [
            ('from .episode_randomization import cup_profile_for_episode',
             'from .episode_randomization import cup_profile_for_episode\n        from scene_assets import selected_cup_asset'),
            ('load_cup_profile('+OLD_CUP+', asset_root)',
             'load_cup_profile(selected_cup_asset(asset_root, episode), asset_root)')],
        'generalization_runtime.py': [
            ('from .episode_randomization import cup_profile_for_episode',
             'from .episode_randomization import cup_profile_for_episode\n        from scene_assets import selected_cup_asset'),
            ('load_cup_profile('+OLD_CUP+', self.spec.asset_root)',
             'load_cup_profile(selected_cup_asset(self.spec.asset_root, self.spec.episode), self.spec.asset_root)')],
        'generalization_env_cfg.py': [
            ('from robot_sim.asset_cfg.objects.cup import load_cup_profile',
             'from robot_sim.asset_cfg.objects.cup import load_cup_profile\n        from scene_assets import selected_cup_asset'),
            ('load_cup_profile('+OLD_CUP+', env.cfg.generalization.asset_root)',
             'load_cup_profile(selected_cup_asset(env.cfg.generalization.asset_root, episode), env.cfg.generalization.asset_root)'),
            ('profile = load_machine_profile(episode.machine_asset, asset_root)',
             'profile = load_machine_profile(episode.machine_asset, asset_root)\n'
             '        from scene_assets import selected_cup_asset\n'
             '        from robot_sim.asset_cfg.objects.cup import load_cup_profile\n'
             '        cup_profile = load_cup_profile(selected_cup_asset(asset_root, episode), asset_root)\n'
             '        cfg.scene.cup.spawn.usd_path = str(resolve_asset(cup_profile.asset_ref, asset_root))'),
            ('        cfg.scene.coffee_machine.init_state.rot = episode.environment_from_machine_root.orientation_wxyz',
             '        cfg.scene.coffee_machine.init_state.rot = episode.environment_from_machine_root.orientation_wxyz\n'
             '        if not profile.buttons:\n'
             '            cfg.scene.coffee_machine.init_state.joint_pos = {}\n'
             '            cfg.scene.coffee_machine.init_state.joint_vel = {}\n'
             '            cfg.scene.coffee_machine.actuators = {}')],
        'episode_scene.py': [
            ('if self.machine_asset not in MACHINE_ASSETS or self.split not in {"development", "evaluation"}:',
             'from scene_assets import validate_episode_machine\n'
             '        if (self.machine_asset not in MACHINE_ASSETS and not validate_episode_machine(self.machine_asset, self.metadata)) or self.split not in {"development", "evaluation"}:')],
        'episode_randomization.py': [
            ('return (episode.machine_asset, episode.randomization.cup_height_m if episode.randomization else None)',
             'from scene_assets import episode_selection\n'
             '    key = (episode.machine_asset, episode.randomization.cup_height_m if episode.randomization else None)\n'
             '    selection = episode_selection(episode)\n'
             '    return (*key, selection["asset_set_digest"]) if selection is not None else key')],
        'initial_robot_state.py': [
            ('metadata = getattr(episode, "metadata", None)\n    if metadata is None:\n        return None',
             'metadata = getattr(episode, "metadata", None)\n'
             '    if isinstance(metadata, dict) and "scene_asset_set" in metadata:\n'
             '        from scene_assets import episode_selection\n'
             '        episode_selection(episode)\n'
             '        metadata = {k: v for k, v in metadata.items() if k != "scene_asset_set"}\n'
             '        if not metadata:\n'
             '            return None\n'
             '    if metadata is None:\n        return None')],
    }
    result = {PACKAGE+name:replacements for name,replacements in task_edits.items()}
    result['source/robot_sim/robot_sim/asset_cfg/objects/coffee_machine.py'] = [
        ('        not buttons\n        or len({b.button_id for b in buttons}) != len(buttons)',
         '        len({b.button_id for b in buttons}) != len(buttons)')]
    return result


def prepare(source, output):
    source, output = Path(source).resolve(strict=True), Path(output)
    if output.exists() or output.is_symlink() or output.resolve().is_relative_to(source):
        raise ValueError('new isolated native output outside the source required')
    patches = {}
    for name, replacements in edits().items():
        path = source/name
        raw = path.read_bytes()
        text = raw.decode()
        for old, new in replacements:
            if text.count(old) != 1:
                raise ValueError('native source contract changed: '+name+' / '+old)
            text = text.replace(old, new)
        compile(text, str(path), 'exec')
        patches[name] = (raw, text.encode())
    shutil.copytree(source, output, symlinks=False,
        ignore=shutil.ignore_patterns('.git', '__pycache__', '*.pyc'))
    receipt = dict(schema='workbench.native_scene_assets.v1', source=str(source), output=str(output), files={})
    for name, (before, after) in patches.items():
        path = output/name
        if path.read_bytes() != before:
            raise ValueError('native source changed while copying: '+name)
        path.write_bytes(after)
        receipt['files'][name] = dict(source_sha256=hashlib.sha256(before).hexdigest(),
            derived_sha256=hashlib.sha256(after).hexdigest())
    (output/'scene-asset-consumer.json').write_text(json.dumps(receipt, indent=2)+'\n')
    return receipt


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(prepare(args.source, args.output)))
