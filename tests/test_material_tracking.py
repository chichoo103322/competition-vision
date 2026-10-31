from pi_vision.material_tracking import MaterialTracker
from pi_vision.types import Material


def material(color=1, x=100, y=80):
    return Material(1, color, 0, x, y, 0., 90)


def test_tracks_follow_new_pixel_positions_without_smoothing_or_reusing_stale_points():
    tracker = MaterialTracker()
    first = tracker.update([material()], 10.)[0]
    moved = tracker.update([material(x=106, y=84)], 10.3)[0]
    assert first.track_id == moved.track_id
    assert moved.tracking_status == 'updated'
    assert (moved.center_x, moved.center_y) == (106, 84)
    assert tracker.update([], 10.4) == []
    recovered = tracker.update([material(x=110, y=85)], 10.6)[0]
    assert recovered.track_id == first.track_id


def test_color_change_long_loss_and_large_jump_create_new_track_ids():
    tracker = MaterialTracker(max_gap_s=.5, max_displacement_px=100)
    original = tracker.update([material()], 1.)[0]
    changed = tracker.update([material(color=2, x=102)], 1.1)[0]
    after_loss = tracker.update([material(x=103)], 2.)[0]
    assert len({original.track_id, changed.track_id, after_loss.track_id}) == 3


def test_ambiguous_two_object_association_does_not_arbitrarily_keep_old_ids():
    tracker = MaterialTracker()
    initial = tracker.update([material(x=80), material(x=120)], 1.)
    crossed = tracker.update([material(x=100), material(x=100)], 1.1)
    assert len({item.track_id for item in initial+crossed}) == 4
