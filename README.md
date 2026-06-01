# DeepC Group Meeting Slides

独立 Marp 汇报仓库。

## 文件结构

- `slides.md`：组会汇报源文件
- `figures/`：本地图片资源
- `data/aggregated_results.csv`：当前汇总结果

## 预览

VSCode 安装 **Marp for VS Code**，打开 `slides.md`。

## 导出 PDF

```bash
npm install
npm run pdf
```

或：

```bash
npx @marp-team/marp-cli slides.md --pdf --allow-local-files
```

## 导出 PPTX

```bash
npm run pptx
```
