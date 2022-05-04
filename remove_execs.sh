find . -name "*.cpp" -type f -executable|while read fname; do
  echo "$fname"
  chmod -x $fname
done
