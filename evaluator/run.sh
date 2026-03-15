#!/bin/bash

# 建立輸出目錄
mkdir -p ../output_pa2

# 執行 floorplanner
echo "Running floorplanner..."
./bin/fp 0.25 ../input_pa2/ami33.block ../input_pa2/ami33.nets ../output_pa2/ami33_25.out 2>&1
./bin/fp 0.5 ../input_pa2/ami33.block ../input_pa2/ami33.nets ../output_pa2/ami33_50.out 2>&1
./bin/fp 0.75 ../input_pa2/ami33.block ../input_pa2/ami33.nets ../output_pa2/ami33_75.out 2>&1

# 執行 evaluator
echo "Running evaluator..."
../evaluator/evaluator.sh ../input_pa2/ami33.block ../input_pa2/ami33.nets ../output_pa2/ami33_25.out 0.25 > ../output_pa2/ami33_25_evaluator.log 2>&1
../evaluator/evaluator.sh ../input_pa2/ami33.block ../input_pa2/ami33.nets ../output_pa2/ami33_50.out 0.5 > ../output_pa2/ami33_50_evaluator.log 2>&1
../evaluator/evaluator.sh ../input_pa2/ami33.block ../input_pa2/ami33.nets ../output_pa2/ami33_75.out 0.75 > ../output_pa2/ami33_75_evaluator.log 2>&1

# 顯示 evaluator 的輸出
# echo "Evaluator output:"
# cat ../output_pa2/ami33_evaluator.log

# 執行 floorplanner
echo "Running floorplanner..."
./bin/fp 0.25 ../input_pa2/ami49.block ../input_pa2/ami49.nets ../output_pa2/ami49_25.out 2>&1
./bin/fp 0.5 ../input_pa2/ami49.block ../input_pa2/ami49.nets ../output_pa2/ami49_50.out 2>&1
./bin/fp 0.75 ../input_pa2/ami49.block ../input_pa2/ami49.nets ../output_pa2/ami49_75.out 2>&1

# 執行 evaluator
echo "Running evaluator..."
../evaluator/evaluator.sh ../input_pa2/ami49.block ../input_pa2/ami49.nets ../output_pa2/ami49_25.out 0.25 > ../output_pa2/ami49_25_evaluator.log 2>&1
../evaluator/evaluator.sh ../input_pa2/ami49.block ../input_pa2/ami49.nets ../output_pa2/ami49_50.out 0.5 > ../output_pa2/ami49_50_evaluator.log 2>&1
../evaluator/evaluator.sh ../input_pa2/ami49.block ../input_pa2/ami49.nets ../output_pa2/ami49_75.out 0.75 > ../output_pa2/ami49_75_evaluator.log 2>&1

# 顯示 evaluator 的輸出
# echo "Evaluator output:"
# cat ../output_pa2/ami49_evaluator.log

# 執行 floorplanner
echo "Running floorplanner..."
./bin/fp 0.25 ../input_pa2/apte.block ../input_pa2/apte.nets ../output_pa2/apte_25.out 2>&1
./bin/fp 0.5 ../input_pa2/apte.block ../input_pa2/apte.nets ../output_pa2/apte_50.out 2>&1
./bin/fp 0.75 ../input_pa2/apte.block ../input_pa2/apte.nets ../output_pa2/apte_75.out 2>&1

# 執行 evaluator
echo "Running evaluator..."
../evaluator/evaluator.sh ../input_pa2/apte.block ../input_pa2/apte.nets ../output_pa2/apte_25.out 0.25 > ../output_pa2/apte_25_evaluator.log 2>&1
../evaluator/evaluator.sh ../input_pa2/apte.block ../input_pa2/apte.nets ../output_pa2/apte_50.out 0.5 > ../output_pa2/apte_50_evaluator.log 2>&1
../evaluator/evaluator.sh ../input_pa2/apte.block ../input_pa2/apte.nets ../output_pa2/apte_75.out 0.75 > ../output_pa2/apte_75_evaluator.log 2>&1

# 顯示 evaluator 的輸出
# echo "Evaluator output:"
# cat ../output_pa2/apte_evaluator.log

# 執行 floorplanner
echo "Running floorplanner..."
./bin/fp 0.25 ../input_pa2/hp.block ../input_pa2/hp.nets ../output_pa2/hp_25.out 2>&1
./bin/fp 0.5 ../input_pa2/hp.block ../input_pa2/hp.nets ../output_pa2/hp_50.out 2>&1
./bin/fp 0.75 ../input_pa2/hp.block ../input_pa2/hp.nets ../output_pa2/hp_75.out 2>&1

# 執行 evaluator
echo "Running evaluator..."
../evaluator/evaluator.sh ../input_pa2/hp.block ../input_pa2/hp.nets ../output_pa2/hp_25.out 0.25 > ../output_pa2/hp_25_evaluator.log 2>&1
../evaluator/evaluator.sh ../input_pa2/hp.block ../input_pa2/hp.nets ../output_pa2/hp_50.out 0.5 > ../output_pa2/hp_50_evaluator.log 2>&1
../evaluator/evaluator.sh ../input_pa2/hp.block ../input_pa2/hp.nets ../output_pa2/hp_75.out 0.75 > ../output_pa2/hp_75_evaluator.log 2>&1

# 顯示 evaluator 的輸出
# echo "Evaluator output:"
# cat ../output_pa2/hp_evaluator.log

# 執行 floorplanner
echo "Running floorplanner..."
./bin/fp 0.25 ../input_pa2/xerox.block ../input_pa2/xerox.nets ../output_pa2/xerox_25.out 2>&1
./bin/fp 0.5 ../input_pa2/xerox.block ../input_pa2/xerox.nets ../output_pa2/xerox_50.out 2>&1
./bin/fp 0.75 ../input_pa2/xerox.block ../input_pa2/xerox.nets ../output_pa2/xerox_75.out 2>&1

# 執行 evaluator
echo "Running evaluator..."
../evaluator/evaluator.sh ../input_pa2/xerox.block ../input_pa2/xerox.nets ../output_pa2/xerox_25.out 0.25 > ../output_pa2/xerox_25_evaluator.log 2>&1
../evaluator/evaluator.sh ../input_pa2/xerox.block ../input_pa2/xerox.nets ../output_pa2/xerox_50.out 0.5 > ../output_pa2/xerox_50_evaluator.log 2>&1
../evaluator/evaluator.sh ../input_pa2/xerox.block ../input_pa2/xerox.nets ../output_pa2/xerox_75.out 0.75 > ../output_pa2/xerox_75_evaluator.log 2>&1

# 顯示 evaluator 的輸出
# echo "Evaluator output:"
# cat ../output_pa2/xerox_evaluator.log

echo "All tests completed. Results are saved in the output directory." 