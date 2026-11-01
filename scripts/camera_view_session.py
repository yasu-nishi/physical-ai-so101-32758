"""Host-operated camera-position experiments; no automatic visual decisions."""
import staged_session

staged_session.runner.STATE = staged_session.runner.ROOT/'tmp/camera_view_session.local.json'
staged_session.runner.CASES = staged_session.runner.ROOT/'config/physical_ai/camera_views/cases.json'

if __name__ == '__main__':
    staged_session.main()
