# 🌱💰 Invest4Better: Sustainable & Profitable Investing App 🚀

## ✨ Project Overview

Welcome to [Invest4Better]("https://invest4better.org/")! This application competed in and won the **Congressional App Challenge** in New York's 18th Congressional District with a clear vision: to empower users to make investment decisions that are not only financially sound but also environmentally responsible. In today's world, it's crucial to align your investments with your values, and Invest4Better makes it easy to find companies that are both good for the planet and good for your wallet.

Our goal is to simplify sustainable investing, showing you how to make money while supporting companies that prioritize environmental well-being. Get ready to invest in a brighter, greener future! 🌍💵

---

## 🎥 TLDR: Invest4Better in Action

_This [link](https://www.youtube.com/watch?v=s2mmniL59G4) is the video submission for the 2024-2025 Congressional App Challenge_

## 💡 Skills & Technologies

This project showcases expertise in:

- **Full-Stack Web Development:** Next.js (Frontend & Backend API Routes)
- **Frontend Development:** React, JavaScript/TypeScript, UI/UX Design
- **Financial Data Integration:** External APIs (Financial Modeling Prep)
- **AI/ML Integration:** OpenAI API, Azure AI Services
- **Data Visualization:** Presenting complex financial & ESG data
- **Sustainable Technologies:** ESG principles, Impact Investing
- **Languages:** JavaScript/TypeScript
- **Version Control:** Git, GitHub

## ✅ Key Benefits & Features

- **🌱 Sustainable Investing:** Discover and invest in companies that demonstrate strong environmental responsibility.

- **💰 Profitability Focus:** Identify companies with solid financial performance, ensuring your investments are also good for your wallet.

- **🔎 Data-Driven Insights:** Leverage financial data and AI to make informed investment decisions.

- **📈 User-Friendly Interface:** Navigate a clean and intuitive platform designed for seamless investing.

- **🌐 Next.js Framework:** Built with modern web technologies for a fast and responsive user experience.

---

## 💻 How It Works

Invest4Better is a dynamic web application that combines financial data with AI insights to help users identify promising sustainable investment opportunities.

1.  **Sector Selection:** Users start by **choosing a specific sector** to explore. This allows them to narrow down their focus.

2.  **Sector Sustainability Overview:** The app visualizes sectors based on their average **ESG (Environmental, Social, and Governance) scores**.

    - **ESG Score:** This rating evaluates a company's performance on sustainability and ethical practices, assessing factors like environmental impact, social responsibility, and governance structures. A higher ESG score typically indicates a company's commitment to responsible business practices.

    - **Interpretation in Invest4Better:** For the purpose of Invest4Better's calculations and scoring, **lower ESG scores are considered better**, indicating less negative impact or stronger positive practices.

    - **Visual Cues:**

      - **Circle Size:** Larger circles indicate a **bigger market capitalization** for companies within that sector.

      - **Color Coding for Sector ESG:** The color of the sector represents the average ESG score of all companies within it, relative to other sectors:

        - **Light Green:** Highly Above Average Sustainability Practices

        - **Dark Green:** Above Average Sustainability Practices

        - **Yellow:** Average Sustainability Practices

        - **Orange:** Questionable Sustainability Practices

        - **Red:** Problematic Sustainability Practices

3.  **Company Selection:** After selecting a sector, the app displays **all S&P 500 companies within that sector**. Users can then **select up to three companies** for detailed analysis.

4.  **I4B Index Calculation:** For the selected companies, the **I4B Index** is calculated and displayed. This proprietary index is determined by the **Net Profit Margin divided by the ESG Score**.

    - **I4B Index Purpose:** It helps users understand how much profit a company generates relative to its environmental impact, with a **higher I4B Index indicating a better balance** of profitability and sustainability.

5.  **Financial Data Fetching:** The application integrates with the **Financial Modeling Prep API** to retrieve up-to-date financial statements, key metrics, and stock data for the selected companies.

6.  **AI Recommendation & Analysis:** Using **OpenAI's API** (and potentially **Azure's AI services** for scalable deployments), the system processes and analyzes the crunch financial data and ESG data for the three selected companies. It then generates a **comprehensive AI-powered recommendation** on which stock to consider buying, weighing both financial performance and ESG factors equally.

7.  **Investment Analysis & Presentation:** The processed financial, environmental, and AI-generated insights are presented to the user in a clear, digestible format, allowing them to compare and evaluate potential investments based on both profit potential and environmental responsibility.

---

## ⚙️ Technical Details

- **Framework:** Next.js

- **Language:** JavaScript/TypeScript (React)

- **Frontend:** React components, styled with modern CSS practices.

- **Backend:** Next.js API Routes for server-side data fetching and AI integration.

- **External APIs:**

  - **Financial Modeling Prep API:** For comprehensive financial data.

  - **OpenAI API:** For AI-driven insights and text analysis.

  - **Azure Account (with credits):** Potentially used for additional AI services or scalable deployments.

---

## 🚀 Getting Started (for Developers)

To clone and run your own instance of Invest4Better:

1.  **Clone the Repository:**

    ```bash
    git clone https://github.com/Hiptostee/Portfolio.git
    cd invest4better
    ```

2.  **Install Dependencies:**

    ```bash
    npm install
    # or
    pnpm install
    # or
    yarn install

    ```

3.  **Obtain API Keys:**

    - **Financial Modeling Prep API Key:** Register on [Financial Modeling Prep](https://site.financialmodelingprep.com/) to get your API key.

    - **OpenAI API Key:** Obtain your API key from [OpenAI](https://platform.openai.com/account/api-keys).

    - **Azure Account:** Ensure you have an Azure account with sufficient credits if your implementation uses Azure services (e.g., Azure OpenAI, Azure Functions, etc.).

4.  **Configure Environment Variables:**
    Create a `.env.local` file in the root of your project (`Portfolio/invest4better.org`) and add your API keys. **Note the specific format for `API_KEYS` for Financial Modeling Prep:**

    ```
    OPENAI_API_KEY=your_openai_api_key
    API_KEYS=[{"api_key": "your_fmp_key_1"}, {"api_key": "your_fmp_key_2"}]
    ```

5.  **Run the Development Server:**

    ```bash
    npm run dev
    # or
    yarn dev
    ```

6.  **Access the App:** Open your browser and navigate to `http://localhost:3000`.

---

This app was made in collaboration with Abhay Gupta (John Jay High School) and Rishabh Nambiar (Binghamton University)
