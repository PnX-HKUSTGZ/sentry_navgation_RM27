import difflib
import subprocess

import pytest

from prepare_pointlio_source import prepare


def test_patch_upgrade_accepts_previous_manifest_and_preserves_unknown_local_edit(tmp_path):
    repository = tmp_path / 'src/point_lio'
    repository.mkdir(parents=True)
    subprocess.run(['git', 'init', '-q', str(repository)], check=True)
    (repository / 'example.cpp').write_text('int value = 1;\n')
    subprocess.run(['git', '-C', str(repository), 'add', 'example.cpp'], check=True)
    subprocess.run(['git', '-C', str(repository), '-c', 'user.name=Test', '-c', 'user.email=test@example.invalid',
                    'commit', '-qm', 'fixture'], check=True)
    revision = subprocess.check_output(['git', '-C', str(repository), 'rev-parse', 'HEAD'], text=True).strip()
    patches = tmp_path / 'tools/global_relocalization/pointlio_patches'
    patches.mkdir(parents=True)
    def patch(name, before, after):
        (patches / name).write_text(''.join(difflib.unified_diff(
            [f'int value = {before};\n'], [f'int value = {after};\n'],
            fromfile='a/example.cpp', tofile='b/example.cpp')))
    patch('0001.patch', 1, 2)
    prepare(tmp_path, revision)
    target = tmp_path / 'tools/global_relocalization/.build/mapping_source/example.cpp'
    assert target.read_text() == 'int value = 2;\n'
    patch('0002.patch', 2, 3)
    prepare(tmp_path, revision)
    assert target.read_text() == 'int value = 3;\n'
    target.write_text('int value = 4;\n')
    with pytest.raises(RuntimeError, match='Unrecognized local mapping-source edit'):
        prepare(tmp_path, revision)
    assert target.read_text() == 'int value = 4;\n'
