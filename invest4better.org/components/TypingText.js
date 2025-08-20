import React, { useEffect, useState } from "react";
import ReactMarkdown from "react-markdown";
import remarkGfm from "remark-gfm"; // GitHub Flavored Markdown support
import rehypeKatex from "rehype-katex"; // For rendering LaTeX math
import styles from "../app/TypingText.module.css";
import remarkMath from "remark-math";
import "katex/dist/katex.min.css";

const TypingText = ({
  markdownText,
  typingSpeed = 1,
  botTyping,
  setIsBotTyping,
  components, // Updated: use components instead of renderers
}) => {
  const [displayedText, setDisplayedText] = useState("");
  const [index, setIndex] = useState(0);
  const newText = markdownText.text;

  // Typing effect
  useEffect(() => {
    if (index < newText.length) {
      const timeout = setTimeout(() => {
        setDisplayedText((prev) => prev + newText[index]);
        setIndex(index + 1);
      }, typingSpeed);

      return () => clearTimeout(timeout);
    } else {
      // Notify that the bot has finished typing
      if (setIsBotTyping) {
        setIsBotTyping(false);
      }
    }
  }, [index, newText, typingSpeed, setIsBotTyping]);

  return (
    <div
      className={`${styles.typingContainer} ${styles.markdown} list-decimal`}
    >
      <ReactMarkdown
        remarkPlugins={[remarkGfm, remarkMath]}
        rehypePlugins={[rehypeKatex]}
        className={styles.typingContainer}
        components={components} // Updated: Use components here
      >
        {displayedText}
      </ReactMarkdown>
    </div>
  );
};

export default TypingText;
