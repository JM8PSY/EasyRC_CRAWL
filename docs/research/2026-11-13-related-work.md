# 関連研究サーベイ: 360°カメラによる床連続性を用いた低コスト2D LiDARフロンティア探索

作成日: 2026-11-13
対象アイデア: 安価な2D LiDAR(約10 m、ガラス・暗色壁・長い廊下で無反射)と slam_toolbox の2D占有格子でフロンティア探索を行うクローラに、民生用360°カメラ(dual fisheye → equirectangular)を追加する。方位ごとに外観ベースの床セグメンテーション(ロボット前方の参照床パッチ。Ulrich & Nourbakhsh 型)で床がどこまで続くかを求め、平面床の幾何 `d = h / tan(俯角)` で距離に変換し、LiDARのヒット点で打ち切る。この「カメラが見た床」を **探索プランナの意思決定用グリッドだけ** に free として書き込み(SLAM地図は汚さない)、LiDARが無反射・死角のときのフロンティア選択の誤りや貪欲さを抑える。

検証方針: 下の表の文献は、タイトル・著者・会場・年をWeb上の一次情報または書誌DB(arXiv, RSS proceedings, IEEE/DBLP, PMLR, 大学リポジトリ)で確認した。確認しきれなかったものには **[未検証]** を付けた。

---

## 1. 要約(切り口ごとの新規性判定)

| 切り口 | 判定 | 根拠(1〜3行) |
|---|---|---|
| ① 360°カメラで安価な2D LiDARの弱点を補ってフロンティア探索を効率化する | **中程度(組み合わせと設定に新規性あり)** | 「距離センサより遠くまで見えるカメラで探索判断を改善する」という考え自体は既にある(Occupancy Anticipation 2020、LRN 2025、WildOS 2026)。一方で、民生用の360°カメラ1台と安価な2D LiDARを組み合わせ、LiDARの無反射やガラスを対象に、カメラ由来の free を意思決定グリッドだけに入れてSLAM地図を汚さない、という組み合わせは見つからなかった。手法そのものより、システムとしての貢献と定量評価で勝負する切り口になる。 |
| ② LiDARを教師にしたオンライン自己教師ありの近→遠の床分類(屋外・雪へ適応) | **低い(手法として新しくない)** | LiDAR/ステレオを教師に近距離の外観を学び、遠距離へ広げる方法は古典的(Dahlkamp+ RSS 2006、Hadsell+ JFR 2009、LAGR)。最近も WVN(RSS 2023)、V-STRONG(ICRA 2024)、STERLING(CoRL 2023)、WayFAST(RA-L 2022、雪を含む)がある。新規性は「屋内の2D LiDAR + 360°カメラ + 雪」という設定と、その結果を探索に使う点に限られ、単独の論文にするには弱い。 |
| ③ カメラで見た床の連続性をフロンティア評価の効用項にして貪欲な選択を減らす | **中程度** | 予測地図の情報利得(MapEx 2025、UPEN 2022、Shrestha 2019)、視覚的な「affordable frontier」(LRN 2025)、VLMの価値マップ(VLFM 2024)と目的は同じ。ただし学習不要・軽量・解釈可能な幾何的手がかり(床がどこまで続くか)を、360°全方位のまま2D LiDARの探索に入れる例は見当たらない。①とまとめるのが最も強い。 |

**結論**: ①と③をまとめて、「低コスト2D LiDAR探索のための、360°カメラによる床連続性の推定(意思決定グリッドへの注入 + 効用項)」とするのが最も有望。②は将来課題か、屋外・雪での補助実験に回すのがよい。

---

## 2. 分野別の関連研究

### 2.1 フロンティア探索とその貪欲性、情報利得、受動的地平線のNBV

| 論文 | 年/会場 | 何をしたか | 本研究との違い | link |
|---|---|---|---|---|
| B. Yamauchi, "A Frontier-Based Approach for Autonomous Exploration" | 1997 / CIRA | 既知の free と未知の境界(フロンティア)へ順に移動して探索する。原点となる手法。 | LiDAR/ソナーだけで動き、最寄りのフロンティアを選ぶ。センサが無反射のときの誤ったフロンティアには対処しない。本研究のベースライン。 | https://ieeexplore.ieee.org/document/613851 |
| D. Holz, N. Basilico, F. Amigoni, S. Behnke, "Evaluating the Efficiency of Frontier-based Exploration Strategies" | 2010 / ISR/ROBOTIK | 最寄りフロンティアなどの単純な戦略を評価し、部屋分割などで経路を短くする拡張を提案。単純な戦略でも高度な戦略に匹敵すると報告。 | 貪欲な選択の評価の枠組みとして参考になる。カメラ情報は使わない。 | https://www.ais.uni-bonn.de/~holz/papers/holz_2010_isr.pdf |
| C. Stachniss, G. Grisetti, W. Burgard, "Information Gain-based Exploration Using Rao-Blackwellized Particle Filters" | 2005 / RSS | 地図と姿勢の不確かさを含めた期待情報利得と移動コストのトレードオフで行動を選ぶ。 | 情報利得を測るのはLiDAR観測モデルだけ。本研究はカメラで見えた床で「利得の見込み」を補正する。 | https://roboticsproceedings.org/rss01/p09.html |
| A. Bircher, M. Kamel, K. Alexis, H. Oleynikova, R. Siegwart, "Receding Horizon 'Next-Best-View' Planner for 3D Exploration" | 2016 / ICRA | RRTを伸ばし、未知体積の利得が最大の枝の最初の辺だけを実行する(受動的地平線のNBV)。 | 3D・UAV向けで、利得は距離センサのレイキャストだけで計算する。 | https://github.com/ethz-asl/nbvplanner |
| C. Cao, H. Zhu, H. Choset, J. Zhang, "TARE: A Hierarchical Framework for Efficiently Exploring Complex 3D Environments" | 2021 / RSS | 局所は密な計画、大域は疎な巡回経路という階層構造で、貪欲な往復を減らす。SubT Final で使われた。 | 3D LiDAR前提で計算も重い。貪欲さを経路最適化で解くのに対し、本研究は観測(カメラの床)で解く。直交するので併用できる。 | https://roboticsproceedings.org/rss17/p018.html |
| B. Zhou, Y. Zhang, X. Chen, S. Shen, "FUEL: Fast UAV Exploration using Incremental Frontier Structure and Hierarchical Planning" | 2021 / IEEE RA-L 6(2) | 増分更新できるフロンティア情報構造と、大域巡回(TSP)+局所精緻化で、従来比3〜8倍速く探索する。 | UAV・深度カメラ向け。貪欲さを巡回順序の最適化で抑える。本研究は効用の推定精度を上げる立場。 | https://arxiv.org/abs/2010.11561 |

### 2.2 地図・占有予測による探索

| 論文 | 年/会場 | 何をしたか | 本研究との違い | link |
|---|---|---|---|---|
| S. K. Ramakrishnan, Z. Al-Halah, K. Grauman, "Occupancy Anticipation for Efficient Exploration and Navigation" | 2020 / ECCV | エゴセントリックなRGB-Dから、見えていない領域の占有を推定(anticipate)して探索とナビゲーションを効率化。Habitat PointNav 2020 で優勝。 | **概念上かなり近い**(画像で深度範囲の外の占有を推定する)。ただし学習ベース・シミュレーション中心・RGB-D前提。本研究は学習不要の幾何推定で、360°カメラ1台と実機の安価な2D LiDARを使う。 | https://arxiv.org/abs/2008.09285 |
| R. Shrestha, F.-P. Tian, W. Feng, P. Tan, R. Vaughan, "Learned Map Prediction for Enhanced Mobile Robot Exploration" | 2019 / ICRA | 生成モデル(VAE)で部分的な2D地図の未知部分を予測し、情報理論的な探索に使う。 | 2D地図だけから予測し、カメラは使わない。本研究は実際に観測した床で未知を埋める。 | https://researchr.org/publication/ShresthaT0TV19 |
| D. S. Chaplot, D. Gandhi, S. Gupta, A. Gupta, R. Salakhutdinov, "Learning to Explore using Active Neural SLAM" | 2020 / ICLR | RGBから地図と姿勢を推定するNeural SLAMと、大域・局所の方策による階層型の探索。 | 学習ベース・シミュレーション中心。本研究は古典的な2D SLAMの上に追加するモジュール。 | https://arxiv.org/abs/2004.05155 |
| G. Georgakis, B. Bucher, A. Arapin, K. Schmeckpeper, N. Matni, K. Daniilidis, "Uncertainty-driven Planner for Exploration and Navigation" (UPEN) | 2022 / ICRA | 視野外の占有を予測し、予測アンサンブルの不確かさが大きい経路を探索に選ぶ。 | 予測の不確かさを利得にする。本研究は観測された床の連続性を利得にする。 | https://arxiv.org/abs/2202.11907 |
| C. Ho, S. Kim, et al., "MapEx: Indoor Structure Exploration with Probabilistic Information Gain from Global Map Predictions" | 2025 / ICRA(arXiv 2409.15590) | LaMaで2D占有地図の大域予測をアンサンブルで行い、分散と可視面積から確率的な情報利得を計算して古典的フロンティア探索を強化。KTHのフロアプランで最寄りフロンティア比25.4%改善。 | **2D LiDAR型フロンティア探索で直接比較できる最有力ベースライン**。予測は地図だけで、観測でもカメラでもない。本研究はカメラで実際に見えた床を使い、LiDARの無反射も補う。 | https://arxiv.org/abs/2409.15590 |
| K. Song et al., "P2 Explore: Efficient Exploration in Unknown Cluttered Environment with Floor Plan Prediction" | 2025 / IROS(arXiv 2409.10878) | 雑然とした2D占有地図からFPUNetでフロアプランを予測し、部屋の順序で探索を誘導。経路長を2.18〜34.60%短縮。 | 地図からの予測だけ。カメラは使わない。 | https://arxiv.org/abs/2409.10878 |

### 2.3 VLM・意味情報によるフロンティア評価、視覚的なフロンティア

| 論文 | 年/会場 | 何をしたか | 本研究との違い | link |
|---|---|---|---|---|
| N. Yokoyama et al., "VLFM: Vision-Language Frontier Maps for Zero-Shot Semantic Navigation" | 2024 / ICRA | 深度から占有地図とフロンティアを作り、RGBとVLM(BLIP-2)で言語に基づく価値マップを作ってフロンティアを評価。Spotで実機実証。 | 目的は物体探索(ObjectNav)。価値は意味情報。本研究は網羅探索で、価値は幾何(床の連続性)。VLMは不要で計算も軽い。 | https://arxiv.org/abs/2312.03275 |
| K. Zhou et al., "ESC: Exploration with Soft Commonsense Constraints for Zero-shot Object Navigation" | 2023 / ICML | LLMの常識を確率的ソフト論理で制約にしてフロンティア選択に使う。 | 物体探索向けの意味的事前知識。本研究とは目的が違う。 | https://arxiv.org/abs/2301.13166 |
| M. Schmittle, R. Baijal, N. Hatch, et al., "Long Range Navigator (LRN): Extending robot planning horizons beyond metric maps" | 2025 / CoRL (PMLR 305) | カメラ画像(SAM2特徴)から「affordable frontier」の方向ヒートマップを学習し、メトリック地図の範囲を超えた長距離の判断を改善。ラベルなし動画で学習。Jetson Orinで約4 Hz。 | **最も近い研究の一つ**。「カメラで地図の地平線の外を見て、フロンティア方向を選ぶ」点が共通。違いは、LRNは屋外・長距離・ゴールあり・学習ベースなのに対し、本研究は屋内の網羅探索・学習不要・360°・2D LiDARの無反射対策・占有格子への明示的な注入であること。 | https://arxiv.org/abs/2504.13149 |
| H. Shah, M. Patel, et al., "WildOS: Open-Vocabulary Object Search in the Wild" | 2026 / IEEE Trans. Field Robotics(arXiv 2602.19308) | 疎なナビグラフのフロンティアノードを、基盤モデルExploRFMが画像空間で予測した走行可能性・視覚的フロンティア・物体類似度で評価。 | 屋外の物体探索。画像空間のフロンティアという考えは近いが、目的とセンサ構成が違う。 | https://arxiv.org/abs/2602.19308 |
| D. Shah, S. Levine, "ViKiNG: Vision-Based Kilometer-Scale Navigation with Geographic Hints" | 2022 / RSS | 画像ベースの局所走行可能性モデルと、俯瞰地図をヒントにするヒューリスティックで、km級のナビゲーションを行う。 | 地図を作らない画像ナビ。網羅探索ではない。 | https://arxiv.org/abs/2202.11271 |

### 2.4 近→遠・自己教師ありの走行可能性推定

| 論文 | 年/会場 | 何をしたか | 本研究との違い | link |
|---|---|---|---|---|
| H. Dahlkamp, A. Kaehler, D. Stavens, S. Thrun, G. Bradski, "Self-supervised Monocular Road Detection in Desert Terrain" | 2006 / RSS | レーザで近距離の平坦地を見つけ、その外観をカメラで学習して遠方へ広げる(検出距離 22 m → 約70 m)。Stanley(DARPA GC優勝)で使われた。 | **切り口②の原型**。本研究の「LiDARで打ち切った床の外観モデル」はこの直系。違いは屋内・360°・2D LiDAR・探索への利用。 | https://roboticsproceedings.org/rss02/p05.html |
| R. Hadsell, P. Sermanet, J. Ben, A. Erkan, M. Scoffier, K. Kavukcuoglu, U. Muller, Y. LeCun, "Learning Long-Range Vision for Autonomous Off-Road Driving" | 2009 / J. Field Robotics 26(2) | LAGRでステレオ(約12 m)を教師にした自己教師あり学習で、5〜100 m超の走行可能性を推定。新しい環境にすぐ適応。 | 近→遠の代表例。屋外オフロード向け。 | https://doi.org/10.1002/rob.20276 |
| J. Frey, M. Mattamala, N. Chebrolu, C. Cadena, M. Fallon, M. Hutter, "Fast Traversability Estimation for Wild Visual Navigation" (WVN) | 2023 / RSS | 自己教師ありのViT特徴とロボット上のオンライン教師で、5分未満の現場学習で走行可能性のセグメンテーションを立ち上げる。 | **オンライン適応の現代版**。②をやるなら比較対象。教師は足の接地経験で、LiDARではない。 | https://arxiv.org/abs/2305.08510 |
| S. Jung, J. Lee, X. Meng, B. Boots, A. Lambert, "V-STRONG: Visual Self-Supervised Traversability Learning for Off-road Navigation" | 2024 / ICRA | SAMのマスクと対照学習で、人の走行データから画像の走行可能性を学習。 | オフロード車両向け。屋内の床には過剰な手法。 | https://arxiv.org/abs/2312.16016 |
| H. Karnan et al., "STERLING: Self-Supervised Terrain Representation Learning from Unconstrained Robot Experience" | 2023 / CoRL (PMLR 229) | ラベルなしのロボット経験から、多モーダルな非対照学習で地形表現を学ぶ。 | 地形の選好を学ぶ。探索とは独立。 | https://proceedings.mlr.press/v229/karnan23a.html |
| M. V. Gasparino et al., "WayFAST: Navigation with Predictive Traversability in the Field" | 2022 / IEEE RA-L(IROS 2022) | トラクション推定を自己教師にして、RGB-Dから走行可能性を予測。雪に覆われた草地でも実験し、幾何だけでは避けにくい雪を回避。 | **雪を扱う数少ない自己教師あり研究**。教師はLiDARではなくトラクション。 | https://arxiv.org/abs/2203.12071 |

### 2.5 外観ベースの床検出、全方位画像の床・自由空間推定

| 論文 | 年/会場 | 何をしたか | 本研究との違い | link |
|---|---|---|---|---|
| I. Ulrich, I. Nourbakhsh, "Appearance-Based Obstacle Detection with Monocular Color Vision" | 2000 / AAAI | 前方の参照領域の色(色相・輝度)ヒストグラムで画素を床/障害物に分類。適応モードでは走行中に学習を続ける。 | 本研究の床セグメンテーションの直接の基盤。床の模様が変わったり障害物が床に似ていたりすると失敗する、という既知の弱点がある。 | https://ri.cmu.edu/?p=16753 |
| L.-F. Posada, K. K. Narayanan, F. Hoffmann, T. Bertram, "Floor Segmentation of Omnidirectional Images for Mobile Robot Visual Navigation" | 2010 / IROS | 全方位画像で床を分割し(PMD深度カメラを教師に)、障害物回避や廊下追従に使う。ICRA 2011 にアンサンブル版もある。 | **全方位の床分割は既にある**。ただし探索の意思決定(フロンティア)には使っていない。 | https://dblp.uni-trier.de/rec/conf/iros/PosadaNHB10.html |
| R. Lukierski, S. Leutenegger, A. J. Davison, "Rapid Free-Space Mapping from a Single Omnidirectional Camera" | 2015 / ECMR | 低コストロボット(Dyson 360 Eye 系)が部屋に入ると円運動し、全方位画像列から2D自由空間の占有地図を作る。レーザ距離計と比較。 | **「全方位カメラ1台で自由空間の2D地図」の先行研究**。LiDARの代わりであって補完ではない。探索プランナとの統合や無反射対策は扱っていない。 | https://spiral.imperial.ac.uk/entities/publication/44a69faa-6d5c-404e-a096-03bf07fd7cea |
| R. Lukierski, S. Leutenegger, A. J. Davison, "Room Layout Estimation from Rapid Omnidirectional Exploration" | 2017 / ICRA | 全方位の再構成を占有格子に変換し、床・壁・天井のラベルとグラフカットで部屋の形を推定。 | 部屋の形の推定が目的。 | https://www.doc.ic.ac.uk/~sleutene/publications/lukierski_etal_icra2017.pdf |

### 2.6 全方位・パノラマカメラとLiDARの融合による探索

| 論文 | 年/会場 | 何をしたか | 本研究との違い | link |
|---|---|---|---|---|
| X. Zhan, S. Zhou, Q. Yang, Y. Zhao, H. Liu, S. C. Ramineni, K. Shimada, "Semantic Exploration and Dense Mapping of Complex Environments using Ground Robot with Panoramic LiDAR-Camera Fusion" | 2025 / IEEE RA-L(arXiv 2505.22880) | 地上ロボットでLiDARとパノラマカメラを融合し、意味的な視点と幾何的な視点を分けてサンプリングする階層型の探索と、密な意味地図を作る。 | **パノラマカメラ + LiDARの探索として最も近い構成**。ただし目的は物体の意味マッピングで、カメラは意味情報の取得に使う。本研究はカメラで LiDARの未知・無反射を幾何的に補う点が違う。安価な2D LiDARも対象外。 | https://arxiv.org/abs/2505.22880 |

### 2.7 ガラス・無反射と、カメラ + 2D LiDARの融合

| 論文 | 年/会場 | 何をしたか | 本研究との違い | link |
|---|---|---|---|---|
| L. Weerakoon, G. S. Herr, J. Blunt, M. Yu, N. Chopra, "Cartographer_glass: 2D Graph SLAM Framework using LiDAR for Glass Environments" | 2022 / arXiv(査読付き会場は未確認) | LiDAR強度のスパイクでガラスを検出し、Cartographerの占有格子に追加。 | LiDARだけで対処し、SLAM地図そのものを修正する。本研究はカメラを使い、SLAMは触らず意思決定グリッドだけを変える。X4 PROで強度を使えるかは要確認。 | https://arxiv.org/abs/2212.08633 |
| "Sensor Data Fusion for a Mobile Robot Using Neural Networks" | 2021 / Sensors(MDPI) | ステレオカメラ・2D LiDAR・超音波をニューラルネットで融合し、ガラスを含む占有格子を作る。 | 障害物(ガラス)を見つけて **occupied** にする方向。本研究は床が続くことを見つけて **free** にする方向で、逆の補い方。 | https://pmc.ncbi.nlm.nih.gov/articles/PMC8749872/ |
| PINMAP(低コスト2D LiDARによるガラス検出と地図化) | DGIST [未検証: 著者・会場・年] | 追加ハードなしで、低コスト2D LiDARだけで透明障害物を検出・地図化すると主張。 | LiDARだけ。 | https://dgist.elsevierpure.com/en/publications/pinmap-a-cost-efficient-algorithm-for-glass-detection-and-mapping/ |
| LiDAR + カメラによるガラス画像セグメンテーションと地図の最適化 | 2023 / Chinese J. Mech. Eng. [未検証: タイトル・著者は出版社ページにアクセスできず未確認] | カメラでガラスを分割し、LiDAR地図を最適化する。 | ガラスを障害物として扱う。 | https://doi.org/10.1186/s10033-023-00902-9 |

### 2.8 雪・屋外の走行可能性

| 論文 | 年/会場 | 何をしたか | 本研究との違い | link |
|---|---|---|---|---|
| E. Alamikkotervo, R. Ojala, A. Seppänen, K. Tammi, "TADAP: Trajectory-Aided Drivable area Auto-labeling with Pre-trained self-supervised features in winter driving conditions" | 2023 / arXiv 2312.12954 | GNSS軌跡から走行可能領域のサンプルを取り、自己教師あり特徴の類似度で冬季画像に自動ラベルを付ける。 | 冬・雪での「参照パッチに似た領域」を広げる考え方は、本研究の参照床パッチ方式と同じ。車両・道路向け。 | https://arxiv.org/abs/2312.12954 |
| D. Baril et al., "Kilometer-scale autonomous navigation in subarctic forests: challenges and lessons learned" | 2022 / Field Robotics 2 | 冬の亜寒帯林で teach-and-repeat を18 km超。雪の積もり方が位置推定を劣化させると報告。 | 雪環境の実機の教訓として参考になる。探索ではない。 | https://arxiv.org/abs/2111.13981 |
| WayFAST(2.4に記載) | 2022 / RA-L | 雪の草地を含む自己教師あり走行可能性。 | 同上。 | https://arxiv.org/abs/2203.12071 |
| Vachmanus et al., RGB-熱画像融合による雪道のセマンティックセグメンテーション | 2021 / IEEE Sensors J. [未検証: 書誌は検索要約でのみ確認] | 北海道の雪データセットでRGBと熱画像を融合。 | 熱カメラが必要。 | (未検証のためリンク省略) |

---

## 3. 最も近い研究トップ5と差別化

| 順位 | 研究 | 近い点 | 差別化のポイント |
|---|---|---|---|
| 1 | **LRN** (Schmittle+, CoRL 2025) https://arxiv.org/abs/2504.13149 | カメラで地図の地平線の外を見て、どのフロンティア方向が有望かを判断する。 | 学習不要の幾何推定(参照床 + 平面床の距離)、360°の全周、屋内の網羅探索、2D LiDARの無反射・ガラスへの明示的な対策、占有格子への定量的な注入(SLAMは汚さない)。LRNは方向ヒートマップだけで距離情報がない点も違う。 |
| 2 | **Occupancy Anticipation** (Ramakrishnan+, ECCV 2020) https://arxiv.org/abs/2008.09285 | 画像で深度範囲の外の占有を推定し、探索を効率化する。 | 学習して「予測」するのではなく、「観測」した床を使う(幻覚がない)。安価な実機と360°カメラで評価し、シミュレーション前提ではない。 |
| 3 | **MapEx** (Ho+, ICRA 2025) https://arxiv.org/abs/2409.15590 | 2D占有格子のフロンティア探索で、最寄りフロンティアの貪欲さを未知領域の推定で改善する。 | MapExは地図だけからの予測で、カメラを使わない。LiDARの無反射は予測でも埋まらない。比較実験の第一候補。MapExの予測と本研究の観測を組み合わせる案もある。 |
| 4 | **Lukierski+ ECMR 2015**(全方位カメラ1台で自由空間マッピング) https://spiral.imperial.ac.uk/entities/publication/44a69faa-6d5c-404e-a096-03bf07fd7cea | 低コストロボット + 全方位カメラで2D自由空間を推定する。 | 本研究はLiDARの「置き換え」ではなく「補完」(LiDARのヒットで打ち切り、未知セルだけ埋める)。探索プランナと統合し、効率を評価する。 |
| 5 | **Dahlkamp+ RSS 2006 / Hadsell+ JFR 2009**(近→遠の自己教師あり) https://roboticsproceedings.org/rss02/p05.html | レーザ/ステレオで打ち切った近距離の外観から、遠くの走行可能面を広げる。 | 屋内・360°・2D LiDAR。目的は走行ではなく探索の意思決定。②を主張する場合は、この系譜に対する差分(屋内の照明・反射、雪)を明確に示す必要がある。 |

次点: Zhan+ RA-L 2025(パノラマLiDAR-カメラ探索。意味マッピングが目的)、Posada+ IROS 2010(全方位画像の床分割。探索には使っていない)、WildOS 2026(画像空間のフロンティア。屋外の物体探索)。

---

## 4. 推奨する切り口と比較すべきベースライン

### 推奨する切り口

**「低コスト2D LiDARのフロンティア探索で、360°カメラの床連続性を意思決定グリッドに注入し、フロンティアの効用項にも使う」**(①+③)。

主張の柱(案):
1. 安価な2D LiDARで起きる無反射(長い廊下、ガラス、暗色壁)が、フロンティア探索に誤ったフロンティア選択・往復・取り残しを生むことを定量的に示す(問題提起)。
2. SLAM地図には入れず意思決定グリッドだけに入れる、という設計の妥当性。SLAM地図にカメラの free を入れた場合と比べて、地図精度とループ閉じ込みへの悪影響を示す。
3. 学習不要・Jetsonで実時間・民生用360°カメラ1台という低コスト性。
4. (補助)床の外観モデルをLiDARで打ち切った領域でオンライン更新する(②の軽量版)。屋外・雪で定性的に示す。

### 比較すべきベースライン

| 種類 | ベースライン | 理由 |
|---|---|---|
| 古典 | 最寄りフロンティア(Yamauchi 1997。ROSでは explore_lite / m-explore など) | 貪欲な選択の基準。 |
| 古典 | コスト-利得型のフロンティア(情報利得 / 距離。Stachniss 2005 や Holz 2010 の型) | 効用項の効果を切り分けるため。 |
| 予測型 | MapEx(コード公開。2D LiDAR + KTHフロアプラン) | 「地図からの予測」と「カメラによる観測」の比較。 |
| アブレーション | LiDARだけ / カメラ由来の free をSLAM地図にも入れる / 前方の単眼だけ(360°なし) / 効用項なしで注入だけ / 注入なしで効用項だけ | 各要素の寄与を示すため。 |
| 代替の距離推定 | 単眼深度基盤モデル(Depth Anything V2 など)の深度で床までの距離を出す | 外観ベースの床分割 + 平面幾何が、軽さ・頑健性で妥当かを示すため。 |
| 参考(余力があれば) | LRN型の方向スコア(学習済み)、VLM価値マップ(VLFM型) | 学習・VLM系との位置づけ。 |

---

## 5. 探索論文でよく使われる評価指標

| 指標 | 内容 | 使っている例 |
|---|---|---|
| 探索面積(体積)と時間・走行距離の関係 | 時間または走行距離に対する既知の free 面積の曲線。曲線下面積でまとめることもある。 | TARE(探索効率 = 平均探索体積/秒)、FUEL、MapEx |
| X%網羅までの時間・距離 | 例: 地上真値の free 面積の90%を探索するまでの時間・経路長。 | FUEL、P2 Explore(経路長の短縮率) |
| 総経路長・探索完了時間 | 探索終了(フロンティア消滅)までの総距離・総時間。 | Holz 2010、P2 Explore |
| 地図精度 | 地上真値地図との IoU / 誤り率、占有・free の適合率と再現率。 | Occupancy Anticipation、UPEN、MapEx |
| 往復・取り残しの指標 | ゴール切替の回数、同じ領域の再訪問、取り残された小領域の数。貪欲さの直接指標。 | (独自に定義して報告されることが多い) |
| 計算時間 | 計画周期、CPU/GPU使用率(Jetsonで実時間か)。 | TARE(計算量50%未満)、LRN(Orinで約4 Hz) |
| 安全性 | 衝突数、人の介入回数、失敗試行率。 | LRN(介入回数)、STERLING |
| 知覚の精度(本研究固有) | カメラの床距離の誤差(LiDARヒット・地上真値との差)、誤って free にしたセル率(特にガラス・段差・黒い床)。 | 本研究で追加すべき |

推奨: シミュレーション(Gazebo / Isaac で、無反射材質を模擬したLiDARモデル)で複数の地図 × 複数の試行を行い平均と分散を示す。実機で長い廊下・ガラス・暗色壁を含む代表的な2〜3環境を示す。

---

## 付記: 未検証・要確認

- PINMAP の著者・会場・年。
- Chinese J. Mech. Eng. 2023 のガラス論文の正確なタイトルと著者(DOI 10.1186/s10033-023-00902-9。出版社ページにアクセスできなかった)。
- Vachmanus et al. 2021(IEEE Sensors J.)の書誌。
- Cartographer_glass の査読付き版の有無。
- MapEx の会場(CMU RI のページで ICRA 2025 と確認。arXiv版との差分は未確認)。
- 検索で見つかった2026年のarXivプレプリント(TravExplorer arXiv 2605.19958、TASG-Explore arXiv 2609.08512、単眼深度で2D LiDARの死角を補う研究など)は本文を確認していないため、表に入れていない。
