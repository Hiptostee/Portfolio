import remarkMath from "remark-math";
import rehypeKatex from "rehype-katex";

const nextConfig = {
  pageExtensions: ["js", "jsx", "mdx"],

  webpack(config, options) {
    config.module.rules.push({
      test: /\.mdx?$/,
      use: [
        options.defaultLoaders.babel,
        {
          loader: "@mdx-js/loader",
          options: {
            remarkPlugins: [remarkMath],
            rehypePlugins: [rehypeKatex],
          },
        },
      ],
    });
    return config;
  },
};

export default nextConfig;
