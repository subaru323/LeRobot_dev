点検の対象が計画書そのもの。書き手が行った照合:
- 本家 lerobot（コミット 53feaef）の docs/source/il_robots.mdx（記録・学習・評価の案内）、lerobot-record --help の表示（既定値）、src/lerobot/configs/train.py（steps, resume）、scripts/lerobot_record.py（resume, private, push_to_hub）、utils/constants.py（HF_LEROBOT_HOME）。
- このパソコン: torch 2.11.0+cpu、cuda False。カメラは内蔵の「Web Camera - HD」だけ。lerobot-train --help で libtorchcodec の警告。
- 実機: tools/prime_goal.py の後に SOFollower.connect() を通し、Goal_Position と Goal_Velocity が残り、6関節の移動が 0 ステップだったこと（フォロワー単体、休止姿勢、1回）。
- 開発チームの資料（LeRobot_dev/dev_docs/team_roles.html、streamlit_screens.html、demo_tool_design.html、easy_lerobot の openspec）の要約（サブエージェントによる）。
記録・学習・評価のコマンドは、実機では1回も通していない。
