"""The simulation transport override is scoped, explicit, and validated."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
from launch import LaunchContext

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('sim_nav_transport', ROOT/'launch/sim_with_nav.launch.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_profile_provisions_large_frames_and_keeps_udp():
    ns = {'f': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
    xml = ET.parse(ROOT/'config/fastdds_large_clouds.xml')
    types = {item.find('f:type', ns).text: item for item in xml.findall('.//f:transport_descriptor', ns)}
    assert 'UDPv4' in types
    assert int(types['SHM'].find('f:segment_size', ns).text) >= 64*1024*1024


def test_launch_override_and_opt_out(tmp_path):
    profile = tmp_path/'dds.xml'
    profile.write_text('<profiles/>')
    context = LaunchContext()
    context.launch_configurations['dds_transport_profile'] = str(profile)
    actions = module._configure_dds_transport(context)
    for action in actions[:2]:
        action.execute(context)
    assert context.environment['FASTRTPS_DEFAULT_PROFILES_FILE'] == str(profile)
    assert context.environment['FASTDDS_DEFAULT_PROFILES_FILE'] == str(profile)
    context.launch_configurations['dds_transport_profile'] = ''
    assert module._configure_dds_transport(context) == []
    context.launch_configurations['dds_transport_profile'] = str(tmp_path/'missing.xml')
    with pytest.raises(RuntimeError, match='does not exist'):
        module._configure_dds_transport(context)
