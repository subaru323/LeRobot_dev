点検の対象は計画書そのもの（書き直した後）。書き手が行ったこと:
- docs/core/guides/計画書の書き方.md の欄に合わせて書き直した。中身の事実は C3 の後の版から移した（C3 の 9 件の対応は、計画書の末尾の表）。
- 欄5 の区間は tools/success_interval.py（Wilson の 95% 区間）の出力を docs/records/success_interval_20261028.txt に控え、表はそこから写した。
- 本家 lerobot（コミット 53feaef）のソースで、記録・学習・評価のコマンドと既定値を C3 のときに確かめた（il_robots.mdx、lerobot_record.py、configs/dataset.py、configs/train.py、configs/policies.py、rollout の configs.py と strategies/core.py、config_so_follower.py）。
- 実機: tools/prime_goal.py の後に SOFollower.connect() を通し、6関節の移動が 0 ステップだったこと（フォロワー単体、休止姿勢、1回）。
記録・学習・評価のコマンドは、実機では1回も通していない。リーダーはまだ組み上がっていない。GPU の型番、カメラの型番、HF の組織の費用は分からない。
