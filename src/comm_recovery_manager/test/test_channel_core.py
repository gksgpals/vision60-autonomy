import pytest

from comm_recovery_manager.channel_core import ChannelPlan


def test_channel_plan_returns_each_candidate_once():
    plan = ChannelPlan(['backup_wifi', 'backup_lte'])

    assert plan.attempt_for(0).channel == 'backup_wifi'
    assert plan.attempt_for(1).channel == 'backup_lte'
    assert plan.attempt_for(2) is None


def test_channel_plan_rejects_duplicate_candidates():
    with pytest.raises(ValueError):
        ChannelPlan(['backup_wifi', 'backup_wifi'])


def test_channel_plan_rejects_negative_attempt_count():
    plan = ChannelPlan(['backup_wifi'])
    with pytest.raises(ValueError):
        plan.attempt_for(-1)
